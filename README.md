from pymavlink import mavutil  # Import pymavlink for MAVLink protocol communication
import cv2  # Import OpenCV for image processing

# Establish connection to the drone via TCP
connection = mavutil.mavlink_connection('tcp:localhost:5762')

# Wait for the drone to send a heartbeat signal to confirm connection
connection.wait_heartbeat()
print("Heartbeat from system %u component %u" % (connection.target_system, connection.target_component))

# Define a dictionary that maps flight modes to their respective IDs
mode_map = {
    "LOITER": 5,
    "AUTO": 3,
    "RTL": 6,
    "MANUAL": 0,
    "GUIDED": 4
}

# Function to change the flight mode of the drone
def set_mode(mode):
    if mode not in mode_map:  # Check if the mode is valid
        print(f"Invalid mode: {mode}")
        return

    # Retrieve the mode ID from the dictionary
    mode_id = mode_map[mode]

    # Send MAVLink command to change the flight mode
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id, 0, 0, 0, 0, 0, 0
    )

    # Wait for acknowledgment from the drone
    msg = connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)

# Function to move the drone to a specific GPS location (latitude, longitude, altitude)
def move_to_location(lat, lon, alt):
    # Send a MAVLink command to move the drone to the specified GPS coordinates
    connection.mav.mission_item_send(
        connection.target_system,
        connection.target_component,
        0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # Use relative altitude frame
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  # Command to move to a waypoint
        2, 0, 0, 0, 0, 0,
        lat, lon, alt  # Latitude, longitude, altitude of the destination
    )

    # Wait for acknowledgment from the drone
    msg = connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)

# Function to detect objects using the camera and move the drone to a specific location
def detect_object_and_move():
    cap = cv2.VideoCapture(0)  # Initialize the webcam
    while True:
        ret, frame = cap.read()  # Capture a frame from the camera
        if not ret:
            break  # Exit loop if no frame is captured

        # Convert the frame from BGR to HSV color space for better color filtering
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define the lower and upper bounds of the color to detect (e.g., green)
        lower_color = (30, 150, 50)
        upper_color = (255, 255, 180)

        # Create a binary mask where the detected color is white and the rest is black
        mask = cv2.inRange(hsv, lower_color, upper_color)

        # Find contours of the detected object in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            # Get the bounding rectangle of the largest contour
            x, y, w, h = cv2.boundingRect(cnt)
            # Draw a rectangle around the detected object
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Calculate the center coordinates of the bounding box
            cx = x + w // 2
            cy = y + h // 2
            print(f"Object detected! Coordinates: ({cx}, {cy})")

            # Move the drone to a predefined location (latitude, longitude, altitude)
            lat, lon, alt = 0, 0, 10  # Example coordinates
            move_to_location(lat, lon, alt)

            # After moving, set the drone to Return-to-Launch (RTL) mode
            set_mode("RTL")
            return  # Exit the loop after moving to the object

        # Show the camera frame with the detected object in a window
        cv2.imshow("Frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):  # Exit loop if 'q' key is pressed
            break

    # Release the camera and close the window
    cap.release()
    cv2.destroyAllWindows()

# Main loop to interact with the user for drone control
while True:
    # Prompt user for input to select a command
    girdi = input("Enter a command (1=Arm, 2=TakeOff, 3=Mode, 4=Move to Object): ")

    if girdi == "1":
        # Send a MAVLink command to arm the drone
        connection.mav.command_long_send(connection.target_system, connection.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        # Wait for acknowledgment from the drone
        msg = connection.recv_match(type='COMMAND_ACK', blocking=True)
        print(msg)

    elif girdi == "2":
        # Send a MAVLink command to take off to an altitude of 10 meters
        connection.mav.command_long_send(connection.target_system, connection.target_component,
                                         mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)
        # Wait for acknowledgment from the drone
        msg = connection.recv_match(type='COMMAND_ACK', blocking=True)
        print(msg)

    elif girdi == "3":
        # Prompt the user to input a flight mode and set the drone's mode
        mode = input("Enter a mode (LOITER, AUTO, RTL, MANUAL, GUIDED): ").upper()
        set_mode(mode)

    elif girdi == "4":
        # Start object detection and move the drone based on the detected object's location
        detect_object_and_move()

    else:
        print("Please enter a valid input: 1, 2, 3, or 4.")
