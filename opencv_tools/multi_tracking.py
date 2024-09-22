import cv2
import sys
from random import randint
import time


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
    tracker_type = "MOSSE"
    multi_tracker = cv2.legacy.MultiTracker_create()
    print("Boxes init::: ", bboxes)

    for bbox in bboxes:
        multi_tracker.add(create_tracker_by_name(tracker_type), frame_init, bbox)
    
    return multi_tracker

def add_tracker(multi_tracker, frame, bbox):
    tracker_type = "CSRT"  # You can make this dynamic if needed
    tracker = create_tracker_by_name(tracker_type)
    
    if tracker is None:
        print("Failed to create tracker!")
        return False  # Indicate that the tracker addition failed
    
    multi_tracker.add(tracker, frame, bbox)
    print(f"Added tracker for bbox: {bbox}")
    return True  # Indicate success

def tracking(multi_tracker, frame_update):
    ok, boxes = multi_tracker.update(frame_update)

    if ok:
        print("Tracking update :::::::::::::::::")
        updated_boxes = []  # Store the updated tracking boxes
        for i, new_box in enumerate(boxes):
            (x, y, w, h) = [int(v) for v in new_box]
            updated_boxes.append((x, y, w, h))
            cv2.rectangle(frame_update, (x, y), (x+w, y+h), (0, 255, 0), 2)
        return frame_update, updated_boxes  # Return the updated frame and boxes
    else:
        return False, None