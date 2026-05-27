#!/usr/bin/python3

import time
import signal
from loguru import logger
import roverlib
import roverlib.rovercom as rovercom

# The main user space program
# this program has all you need from roverlib: service identity, reading, writing and configuration
def run(service : roverlib.Service, configuration : roverlib.ServiceConfiguration):
    #
    # Get configuration values
    #
    if configuration is None:
        raise ValueError("Configuration cannot be accessed")
    
    #
    # Access the to the identity of this service, who am I?
    #
    logger.info(f"Hello World, a new Python service {service.name} was born at version {service.version}")

    #
    # Access the service configuration, to use runtime parameters
    #
    tunable_speed = configuration.GetFloatSafe("speed")
    if tunable_speed is None:
        raise ValueError("Failed to get configuration")
    logger.info(f"Fetched runtime configuration example tunable number: {tunable_speed}")


    #
    # Reading from an input, to get data from other services (see service.yaml to understand the input name)
    #
    read_stream : roverlib.ReadStream = service.GetReadStream("imaging", "path")
    if read_stream is None:
        raise ValueError("Failed to get read stream")
    
    #
    # Writing to an output that other services can read (see service.yaml to understand the output name)
    #
    write_stream : roverlib.WriteStream = service.GetWriteStream("decision")
    if write_stream is None:
        raise ValueError("Failed to create write stream 'decision'")

    while True:
        #
        # Reading one message from the stream
        #
        data = read_stream.Read()
        if data is None:
            raise ValueError("Failed to read from 'imaging' service")
        
        # When did the imaging service create this message
        created_at = data.timestamp
        logger.info(f"Received message with timestamp: {created_at}")

        # Get the imaging data
        imaging_data = data.camera_output
        if imaging_data is None:
            return ValueError("Message does not contain camera output. What did imaging do??")

        logger.info(f"Imaging service captured a {imaging_data.resolution.width} by {imaging_data.resolution.height} image")

        # If the imaging has detected any track pieces, the print the X and Y coordinates of 
        # the middle point of the track.
        if len(imaging_data.horizontal_scans) > 0:
            logger.info(f"Track edge found! Left edge X: {imaging_data.horizontal_scans[0].x_left}, right edge X: {imaging_data.horizontal_scans[0].x_right}. Both are at Y: {imaging_data.horizontal_scans[0].y}")

        # This value holds the steering position that we want to pass to the servo (-1 = left, 0 = center, 1 = right)
        steer_position = -0.5

        write_stream.Write(rovercom.SensorOutput(
            sensor_id=1,
            timestamp=int(time.time() * 1000),
            controller_output=rovercom.ControllerOutput(
                steering_angle = steer_position,
                left_throttle = tunable_speed,
                right_throttle = tunable_speed,
            )
        ))

        #
		# Now do something else fun, see if our "tunable_speed" is updated
		#
        logger.info("Checking for tunable number update")

        curr = tunable_speed

        # We are not using the safe version here, because using locks is boring
		# (this is perfectly fine if you are constantly polling the value)
		# nb: this is not a blocking call, it will return the last known value
        new_val = configuration.GetFloat("speed")
        if new_val is None:
            raise ValueError("Failed to get updated tunable number")
        if new_val != curr:
            logger.info(f"Tunable number updated: {curr} -> {new_val}")
            curr = new_val
        tunable_speed = curr

        


# This function gets called when the pipeline gets terminated. This can happen if any other service
# in the pipeline crashes or if the pipeline is stopped via the the web interface.
def on_terminate(sig : signal):
    logger.info(f"signal: {str(sig)}, Terminating service")

    #
    # ...
    # If you need to add clean up logic, add it here
    # ...
    #

    return None

if __name__ == "__main__":
    roverlib.Run(run, on_terminate)
