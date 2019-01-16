import io
import socket
import struct
import time
import picamera

# START A CLIENT SESSION TO SERVER ---------------------------------------------
client_socket = socket.socket()
    # In this case, server address will be my computer
client_socket.connect(('192.168.1.34', 8000))

# Make a file-like object out of the connection
connection = client_socket.makefile('wb')

try:
    # INITIALIZE PI CAMERA -----------------------------------------------------
    camera = picamera.PiCamera()
    camera.resolution = (640, 480)
    # Start a preview and let the camera warm up for 2 seconds
    camera.start_preview()
    time.sleep(2)
    # Note the start time
    start = time.time()

    # START SENDING DATA -------------------------------------------------------
            # This program is design to send first the length of the image as a
            # 32-bit integer (in Little Endian format), then this will be
            # followed by the bytes of image data.

            #          IMAGE LENGTH (4 bytes) + IMAGE DATA (# bytes)

            # A value of length = 0 means the sending of data is finished

    # Construct a stream to hold image data temporarily
    stream = io.BytesIO()

    for foo in camera.capture_continuous(stream, 'jpeg'):
        # Write the length of the capture to the stream and flush to
        # ensure it actually gets sent
        connection.write(struct.pack('<L', stream.tell()))
        connection.flush()
        # Rewind the stream and send the image data over the wire
        stream.seek(0)
        connection.write(stream.read())

        # If we've been capturing for more than 30 seconds, quit (temporarly)
        if time.time() - start > 30:
            break
        # Reset the stream for the next capture
        stream.seek(0)
        stream.truncate()
    # Write a length of zero to the stream to signal we're done
    connection.write(struct.pack('<L', 0))

# CLOSE CONNECTION -------------------------------------------------------------
finally:
    connection.close()
    client_socket.close()
