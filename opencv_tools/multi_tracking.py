import cv2
import sys
from random import randint
import time
from . import utils

utils = utils.Utils()

tracker_types = ['MOSSE', 'CSRT']

def create_tracker_by_name(tracker_type):
    if tracker_type == tracker_types[0]:
        tracker = cv2.legacy.TrackerMOSSE_create()
    elif tracker_type == tracker_types[1]:
        tracker = cv2.legacy.TrackerCSRT_create()
    else:
        tracker = None
        print("Invalid tracker algorithm!")

    return tracker

def getRandomColor():
    return (randint(0, 255), randint(0, 255), randint(0, 255))


def get_multi_tracker(bboxes, frame_init):
    tracker_type = "CSRT"
    multi_tracker = cv2.legacy.MultiTracker_create()
    print("Boxes init::: ", bboxes)

    for bbox in bboxes:
        multi_tracker.add(create_tracker_by_name(tracker_type), frame_init, bbox)
    
    return multi_tracker

def tracking(self, multi_tracker, frame_update):
    ok, boxes = multi_tracker.update(frame_update)

    if ok:      
        updated_boxes = []  # Store the updated tracking boxes

        for i, new_box in enumerate(boxes):
            (x, y, w, h) = [int(v) for v in new_box]
            updated_boxes.append((x, y, w, h))
            cv2.rectangle(frame_update, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.putText(frame_update, f"{x, y, w, h}", (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
            if len(self.old_boxes) > 0:
                old_boxes = self.old_boxes[0]
                utils.calculate_velocity(old_boxes, new_box, 0.02, frame_update)

        self.old_boxes = updated_boxes
        return frame_update, updated_boxes  # Return the updated frame and boxes
    else:
        return False, None