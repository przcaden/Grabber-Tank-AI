
#########################################################################
#                                                                       #
#                      Grabber-Tank PC Client Script                    #
#                              Caden Perez                              #
#                                                                       #
#                          CSCE-480 Intro to AI                         #
#                              Final Project                            #
#                                                                       #
#########################################################################

import cv2
import io
import socket
import struct
from PIL import Image

# Attach server connection to robot over local wifi
machine_name = socket.gethostname()
IPV4 = socket.gethostbyname(machine_name)
print('On network IP ' + IPV4)
port = 5000
server_socket = socket.socket()
server_socket.bind((IPV4, port))
server_socket.listen(0)

# Accept a single connection and make a file-like object out of it
connection = server_socket.accept()[0].makefile('rb')
print('connection accepted')

# Run unending connection to robot and display its RPi video
try:
    img = None
    while True:
        # Read the length of the image as a 32-bit unsigned int. If the
        # length is zero, quit the loop
        image_len = struct.unpack('<L', connection.read(struct.calcsize('<L')))[0]
        if not image_len:
            break
        
        # Start a stream to hold image data and read from it
        image_stream = io.BytesIO()
        image_stream.write(connection.read(image_len))

        # Rewind the stream and open it as an image
        image_stream.seek(0)
        image = Image.open(image_stream)
        
        # Display received image
        if img is None:
            img = cv2.imshow('Tank-Grabber Output', image)
        else:
            img.set_data(image)

        print('Image is %dx%d' % image.size)
        image.verify()
        print('Image is verified')

finally:
    # Close connection
    connection.close()
    server_socket.close()

    # Display a handsome gentleman
    cv2.imshow('Tank-Grabber Output', cv2.imread('assets/close.jpg'))

    # Await inevitable destruction
    cv2.waitKey(0)
    cv2.destroyAllWindows()