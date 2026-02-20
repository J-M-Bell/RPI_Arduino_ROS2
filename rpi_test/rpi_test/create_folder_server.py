#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import CreateFolder
import os

HOME = os.path.expanduser("~") # Get the path to the user's home directory, e.g., /home/username.

class CreateFolderNode(Node):
    """
    A ROS2 node that provides a service to create a folder in the user's home directory.
    The node initializes a service named "create_folder" that accepts requests containing a folder name.
    """
    def __init__(self):
        """
        Initializes the CreateFolderNode and sets up the "create_folder" service.
        """
        super().__init__("create_folder_server")
        self.srv = self.create_service(CreateFolder, "create_folder", self.create_folder_callback)
        self.get_logger().info("Create Folder Service is ready to receive requests.")

    def create_folder_callback(self, request, response):
        """
        A callback function that is called when a request is received on the "create_folder" service.
        The function attempts to create a folder with the specified name in the user's home directory.
        It handles cases where the folder already exists or if there is an error during folder creation,
        and returns an appropriate response message indicating the success or failure of the operation.

            Args:
                request (CreateFolder.Request): The request object containing the folder name 
                                                to be created.
                response (CreateFolder.Response): The response object to be populated with the result
                                                  of the folder creation.
            Returns:
                CreateFolder.Response: The response object containing the success status and message
                                       regarding the folder creation operation.
        """
        folder_name = request.folder_name
        folder_path = os.path.join(HOME, folder_name)

        # Create the folder and handle exceptions for existing folders or other errors
        try:
            os.makedirs(folder_path)
            response.success = True
            response.message = f"Folder '{folder_name}' created successfully at {folder_path}."
        except FileExistsError:
            response.success = False
            response.message = f"Folder '{folder_name}' already exists at {folder_path}."
        except Exception as e:
            response.success = False
            response.message = f"Failed to create folder '{folder_name}': {str(e)}"

        return response


def main(args=None):
    rclpy.init(args=args)
    node = CreateFolderNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
