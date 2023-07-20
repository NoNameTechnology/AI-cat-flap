import cv2
import numpy as np
from tflite_support.task import processor

# Import necessary modules for accessing the GPIO pins and controlling the servo
import RPi.GPIO as GPIO
import time

_MARGIN = 10  # pixels
_ROW_SIZE = 10  # pixels
_FONT_SIZE = 1
_FONT_THICKNESS = 1
_TEXT_COLOR = (0, 0, 255)  # red


# Set the pin numbering mode
GPIO.setmode(GPIO.BCM)

# Set the GPIO pin that the servo is connected to
servo_pin = 23

# Set the servo pin as output
GPIO.setup(servo_pin, GPIO.OUT)

# Create a PWM instance with a frequency of 50 Hz
pwm = GPIO.PWM(servo_pin, 50)

# Start the PWM with a duty cycle of 7.5% (close position)
pwm.start(7.5)

def open_servo():
    # Change duty cycle to 12.5% to open the servo
    pwm.ChangeDutyCycle(12.5)

    # Delay for a short time
    time.sleep(2)

def close_servo():
    # Change duty cycle to 7.5% to close the servo
    pwm.ChangeDutyCycle(7.5)

    # Delay for a short time
    time.sleep(0.2)
    pwm.ChangeDutyCycle(0)

servo_state = False

def visualize(
    image: np.ndarray,
    detection_result: processor.DetectionResult,
) -> np.ndarray:
  """Draws bounding boxes on the input image and return it.

  Args:
    image: The input RGB image.
    detection_result: The list of all "Detection" entities to be visualize.

  Returns:
    Image with bounding boxes.
  """
  global servo_state
  detection_made = False
  for detection in detection_result.detections:
    detection_made = True
    # Draw bounding_box
    bbox = detection.bounding_box
    start_point = bbox.origin_x, bbox.origin_y
    end_point = bbox.origin_x + bbox.width, bbox.origin_y + bbox.height
    cv2.rectangle(image, start_point, end_point, _TEXT_COLOR, 3)

    # Draw label and score
    category = detection.categories[0]
    category_name = category.category_name #zuständig für name anzeigen
    if category_name == "cat+mouse":
        # Close the servo if it is open
        if servo_state:
            close_servo()
            servo_state = False
    elif category_name == "cat":
        probability = round(category.score, 2)
        if probability > 0.5:
            # Open the servo if it is closed
            if not servo_state:
                open_servo()
                servo_state = True
    else:
        # Close the servo if it is open
        if servo_state:
            close_servo()
            servo_state = False
    probability = round(category.score, 2)
    result_text = category_name + ' (' + str(probability) + ')'
    text_location = (_MARGIN + bbox.origin_x,
                     _MARGIN + _ROW_SIZE + bbox.origin_y)
    cv2.putText(image, result_text, text_location, cv2.FONT_HERSHEY_PLAIN,
                _FONT_SIZE, _TEXT_COLOR, _FONT_THICKNESS)
  if not detection_made:
      if servo_state:
            close_servo()
            servo_state = False
  return image
