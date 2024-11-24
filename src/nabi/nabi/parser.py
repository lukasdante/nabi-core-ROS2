import os
import uuid
import json
from pathlib import Path
from dotenv import load_dotenv

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from rcl_interfaces.msg import ParameterDescriptor

from google.cloud.dialogflowcx_v3beta1.services.agents import AgentsClient
from google.cloud.dialogflowcx_v3beta1.services.sessions import SessionsClient
from google.cloud.dialogflowcx_v3beta1.types import session
from google.api_core.client_options import ClientOptions
from google.protobuf.json_format import MessageToDict

class Parser(Node):
    def __init__(self):
        super().__init__('parser')

        try:
            self.project_id = os.getenv("DFCX_PROJECT_ID")
            self.location_id = os.getenv("DFCX_LOCATION_ID")
            self.agent_id = os.getenv("DFCX_AGENT_ID")
            self.service_account = str(Path(__file__).parent.parent.parent.parent) + '/json/' + os.getenv("DFCX_SERVICE_ACCOUNT")
            os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = self.service_account
            self.agent = f'projects/{self.project_id}/locations/{self.location_id}/agents/{self.agent_id}'
            self.session_id = None

            self.declare_parameter('language', 'en-US',
                                   ParameterDescriptor(description='Language of the transcription input.'))

            self.language = self.get_parameter('language').get_parameter_value().string_value
 
            self.response_publisher = self.create_publisher(String, 'conversation/response', 10)
            self.params_publisher = self.create_publisher(String, 'conversation/parameters', 10)
            self.subscriber = self.create_subscription(String, 'conversation/transcript', self.detect_intent, 10)

            self.get_logger().info("Parser initialized.")
        except Exception as e:
            self.get_logger().error(f"Unable to initialize parser: {e}")

    def detect_intent(self, msg):
        """ Detect intent, extract parameters, and output response. """

        if not self.session_id:
            self.session_id = uuid.uuid4()
        
        session_path = f'{self.agent}/sessions/{self.session_id}'
        api_endpoint = f'{AgentsClient.parse_agent_path(self.agent)["location"]}-dialogflow.googleapis.com:443'
        client_options = ClientOptions(api_endpoint=api_endpoint)
        session_client = SessionsClient(client_options=client_options)

        # Configure the request
        text_input = session.TextInput(text=msg.data)
        query_input = session.QueryInput(text=text_input, language_code=self.language)
        
        # Send the request
        request = session.DetectIntentRequest(
            session=session_path, query_input=query_input
        )

        # Obtain the response
        response = session_client.detect_intent(request=request)

        # 
        response_messages = [
            " ".join(msg.text.text) for msg in response.query_result.response_messages
        ]

        # Convert the parameters to a dictionary and prepare response
        parameters = MessageToDict(response._pb)
        self.get_logger().info(f"Parameters: {parameters['queryResult']['parameters']}")

        # Prepare parameters message data
        params_msg = String()
        params_msg.data = json.dumps(parameters)

        # Publish parameters message
        self.params_publisher.publish(params_msg)
        self.get_logger().info(f"Parameters: {parameters['queryResult']['parameters']}")

        # Prepare response message data
        response_text = ' '.join(response_messages)
        response_msg = String()
        response_msg.data = response_text

        # Publish response message
        self.response_publisher.publish(response_msg)
        self.get_logger().info(f"Response text: {response_text}")
        
        # Return the Intent
        return {'response': response_text, 'parameters': parameters['queryResult']['parameters']}


def main(args=None):
    load_dotenv()

    try:
        rclpy.init(args=args)

        lone_parser = Parser()

        rclpy.spin(lone_parser)

    except KeyboardInterrupt:
        pass

    lone_parser.destroy_node()


if __name__ == '__main__':
    main()