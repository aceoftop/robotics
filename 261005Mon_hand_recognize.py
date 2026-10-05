import serial
import time
import cv2
import mediapipe as mp
import math

#DEFINE
PORT = '/dev/ttyACM0'
BAUDRATE = 9600

STATE_LEFT      = 1
STATE_RIGHT     = 2
STATE_UP        = 3
STATE_DOWN      = 4
STATE_CLOSE     = 5
STATE_OPEN      = 6
STATE_FINGER2   = 7
STATE_FINGER3   = 8
STATE_INIT      = 9

prev_state = 0

cnt_left = 0
cnt_right = 0
cnt_up = 0
cnt_down = 0
cnt_close = 0
cnt_open = 0
cnt_finger2 = 0
cnt_finger3 = 0
cnt_init    = 0

#serial
ser = serial.Serial(PORT, BAUDRATE, timeout=1)
print(f"[{PORT}] serial start (speed:{BAUDRATE})")
time.sleep(2)

#hand
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
hands = mp_hands.Hands(max_num_hands=2, min_detection_confidence=0.7)

cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 320)

last_tx_time = 0

while cap.isOpened():
    success, image = cap.read()
    if not success:
        print("Can't camera!")
        continue

    image = cv2.flip(image,1)
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    results = hands.process(image_rgb)

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:

            #Extract coordinates for point 0(Wrist) and point 9(base of the middle finger)
            w_x, w_y = hand_landmarks.landmark[0].x, hand_landmarks.landmark[0].y
            m_x, m_y = hand_landmarks.landmark[9].x, hand_landmarks.landmark[9].y
            hand_size = math.sqrt((w_x - m_x)**2 + (w_y - m_y)**2)
            
            #Get index finger joint coordinates(X,Y)
            thumb_mcp = hand_landmarks.landmark[1]
            thumb_tip = hand_landmarks.landmark[4]

            index_mcp = hand_landmarks.landmark[5] #start
            index_pip = hand_landmarks.landmark[6]
            index_dip = hand_landmarks.landmark[7]
            index_tip = hand_landmarks.landmark[8] # end
            
            #Middle finger joint coordinates (for checking flexion)
            middle_mcp = hand_landmarks.landmark[9]
            middle_tip = hand_landmarks.landmark[12]
            
            ring_mcp = hand_landmarks.landmark[13]
            ring_tip = hand_landmarks.landmark[16]

            pinky_mcp = hand_landmarks.landmark[17]
            pinky_tip = hand_landmarks.landmark[20]


            #===== chose =====
            #left
            is_pointing_left_X = (index_mcp.x - index_tip.x) > (hand_size * 0.5)
            #rifht
            is_pointing_right_X = (index_tip.x - index_mcp.x) > (hand_size * 0.5)
            #up
            is_pointing_up_Y = (index_mcp.y - index_tip.y) > (hand_size * 0.5)
            #down
            is_pointing_down_Y = (index_tip.y - index_mcp.y) > (hand_size * 0.5)
            #close
            is_index_folded = index_tip.y > index_mcp.y
            is_middle_folded = middle_tip.y > middle_mcp.y
            is_ring_folded = ring_tip.y > ring_mcp.y
            is_pinky_folded = pinky_tip.y > pinky_mcp.y
            is_thumb_folded = abs(thumb_tip.x - thumb_mcp.x) < (hand_size * 0.4)
            
            is_fist = is_index_folded and is_middle_folded and is_ring_folded and is_pinky_folded and is_thumb_folded

            #open
            is_thumb_open = (thumb_mcp.y - thumb_tip.y) > (hand_size * 0.2)
            is_index_open = (index_mcp.y - index_tip.y) > (hand_size * 0.2)
            is_middle_open = (middle_mcp.y - middle_tip.y) > (hand_size * 0.2)
            is_ring_open = (ring_mcp.y - ring_tip.y) > (hand_size * 0.2)
            is_pinky_open = (pinky_mcp.y - pinky_tip.y) > (hand_size * 0.2)

            is_hand_open = is_thumb_open and is_index_open and is_middle_open and is_ring_open and is_pinky_open

            #share
            is_vertical_X = abs(index_tip.x - index_mcp.x) < (hand_size * 0.3)
            is_horizontal_Y = abs(index_tip.y - index_mcp.y) < (hand_size * 0.3)
            
            # 2 fingers ( index, middle )
            is_two_fingers_raised =  is_index_open and is_middle_open and is_ring_folded and is_pinky_folded and is_thumb_folded

            # 3 finger ( index, middle, ring )
            is_three_fingers_raised =  is_index_open and is_middle_open and is_ring_open and is_pinky_folded and is_thumb_folded

            # 4 finger ( index, middle, ring, pinky )
            is_four_fingers_raised = is_index_open and is_middle_open and is_ring_open and is_pinky_open and is_thumb_folded

            #===== final chose =====
            #left
            if is_pointing_left_X and is_horizontal_Y and is_middle_folded:
                cv2.putText(image, "LEFT", (30, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                #print("Left")
                current_time = time.time()
                if current_time - last_tx_time >= 1.0:
                    if prev_state is not None and prev_state != STATE_LEFT:
                        cnt_left = 0
                        prev_state = STATE_LEFT

                    print("Left:", cnt_left)
                    cnt_left += 1

                    tx_message = 'l'
                    ser.write(tx_message.encode('utf-8'))
                    last_tx_time = current_time
            
            #right
            elif is_pointing_right_X and is_horizontal_Y and is_middle_folded:
                cv2.putText(image, "RIGHT", (30, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                #print("Right")
                current_time = time.time()
                if current_time - last_tx_time >= 1.0:
                    if prev_state is not None and prev_state != STATE_RIGHT:
                        cnt_right = 0
                        prev_state = STATE_RIGHT

                    print("Right:", cnt_right)
                    cnt_right += 1
                    tx_message = 'r'
                    ser.write(tx_message.encode('utf-8'))
                    last_tx_time = current_time

            #up
            elif is_pointing_up_Y and is_vertical_X and is_middle_folded:
                cv2.putText(image, "UP", (30, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                #print("UP")
                current_time = time.time()
                if current_time - last_tx_time >= 1.0:
                    if prev_state is not None and prev_state != STATE_UP:
                        cnt_up = 0
                        prev_state = STATE_UP

                    print("Up:", cnt_up)
                    cnt_up += 1
                    tx_message = 'u'
                    ser.write(tx_message.encode('utf-8'))
                    last_tx_time = current_time

            #down
            elif is_pointing_down_Y and is_vertical_X and is_middle_folded:
                cv2.putText(image, "DOWN", (30, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                #print("DWON")
                current_time = time.time()
                if current_time - last_tx_time >= 1.0:
                    if prev_state is not None and prev_state != STATE_DOWN:
                        cnt_down = 0
                        prev_state = STATE_DOWN

                    print("Down:", cnt_down)
                    cnt_down += 1
                    tx_message = 'd'
                    ser.write(tx_message.encode('utf-8'))
                    last_tx_time = current_time
            
            #close
            elif is_fist:
                cv2.putText(image, "Close!", (30, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                #print("Close")
                current_time = time.time()
                if current_time - last_tx_time >= 1.0:
                    if prev_state is not None and prev_state != STATE_CLOSE:
                        cnt_close = 0
                        prev_state = STATE_CLOSE

                    print("close:", cnt_close)
                    cnt_close += 1;
                    tx_message = 'c'
                    ser.write(tx_message.encode('utf-8'))
                    last_tx_time = current_time

            #open
            elif is_hand_open:
                cv2.putText(image, "Open", (30, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                #print("Open")
                current_time = time.time()
                if current_time - last_tx_time >= 1.0:
                    if prev_state is not None and prev_state != STATE_OPEN:
                        cnt_open = 0
                        prev_state = STATE_OPEN

                    print("open:", cnt_open)
                    cnt_open += 1;
                    tx_message = 'o'
                    ser.write(tx_message.encode('utf-8'))
                    last_tx_time = current_time



            # 2 fingers
            elif is_two_fingers_raised:
                cv2.putText(image, "2 fingers", (30, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                #print("2 fingers")
                current_time = time.time()
                if current_time - last_tx_time >= 1.0:
                    if prev_state is not None and prev_state != STATE_FINGER2:
                        cnt_finger2 = 0
                        prev_state = STATE_FINGER2

                    print("finger2:", cnt_finger2)
                    cnt_finger2 += 1
                    tx_message = '4'
                    ser.write(tx_message.encode('utf-8'))
                    last_tx_time = current_time



            # 3 fingers
            elif is_three_fingers_raised:
                cv2.putText(image, "3 fingers", (30, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                #print("3 fingers")
                current_time = time.time()
                if current_time - last_tx_time >= 1.0:
                    if prev_state is not None and prev_state != STATE_FINGER3:
                        cnt_finger3 = 0
                        prev_state = STATE_FINGER3

                    print("finger3:", cnt_finger3)
                    cnt_finger3 += 1
                    tx_message = '5'
                    ser.write(tx_message.encode('utf-8'))
                    last_tx_time = current_time
            


            # 4 fingers ( thumb is not checked )
            elif is_four_fingers_raised:
                cv2.putText(image, "Init", (30, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                #print("Init")
                current_time = time.time()
                if current_time - last_tx_time >= 1.0:
                    if prev_state is not None and prev_state != STATE_INIT:
                        cnt_init = 0
                        prev_state = STATE_INIT

                    print("Init:", cnt_init)
                    cnt_init += 1;
                    tx_message = 'i'
                    ser.write(tx_message.encode('utf-8'))
                    last_tx_time = current_time
            


            #image drawing
            mp_drawing.draw_landmarks(
                    image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

    cv2.imshow('RaspberryPi5 Hand Tracking', image)

    if cv2.waitKey(5) & 0xFF == ord('q'):
        break

if 'ser' in locals() and ser.is_open:
    ser.close()
    print("serial closed")

cap.release()
cv2.destroyAllWindows()
