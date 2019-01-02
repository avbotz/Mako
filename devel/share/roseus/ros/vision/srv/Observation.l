;; Auto-generated. Do not edit!


(when (boundp 'vision::Observation)
  (if (not (find-package "VISION"))
    (make-package "VISION"))
  (shadow 'Observation (find-package "VISION")))
(unless (find-package "VISION::OBSERVATION")
  (make-package "VISION::OBSERVATION"))
(unless (find-package "VISION::OBSERVATIONREQUEST")
  (make-package "VISION::OBSERVATIONREQUEST"))
(unless (find-package "VISION::OBSERVATIONRESPONSE")
  (make-package "VISION::OBSERVATIONRESPONSE"))

(in-package "ROS")





(defclass vision::ObservationRequest
  :super ros::object
  :slots (_task ))

(defmethod vision::ObservationRequest
  (:init
   (&key
    ((:task __task) 0)
    )
   (send-super :init)
   (setq _task (round __task))
   self)
  (:task
   (&optional __task)
   (if __task (setq _task __task)) _task)
  (:serialization-length
   ()
   (+
    ;; int8 _task
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _task
       (write-byte _task s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _task
     (setq _task (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _task 127) (setq _task (- _task 256)))
   ;;
   self)
  )

(defclass vision::ObservationResponse
  :super ros::object
  :slots (_x _y ))

(defmethod vision::ObservationResponse
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:serialization-length
   ()
   (+
    ;; float32 _x
    4
    ;; float32 _y
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass vision::Observation
  :super ros::object
  :slots ())

(setf (get vision::Observation :md5sum-) "8a3ea109914aae69afdec8f4e1e8c6ca")
(setf (get vision::Observation :datatype-) "vision/Observation")
(setf (get vision::Observation :request) vision::ObservationRequest)
(setf (get vision::Observation :response) vision::ObservationResponse)

(defmethod vision::ObservationRequest
  (:response () (instance vision::ObservationResponse :init)))

(setf (get vision::ObservationRequest :md5sum-) "8a3ea109914aae69afdec8f4e1e8c6ca")
(setf (get vision::ObservationRequest :datatype-) "vision/ObservationRequest")
(setf (get vision::ObservationRequest :definition-)
      "int8 task
---
float32 x
float32 y

")

(setf (get vision::ObservationResponse :md5sum-) "8a3ea109914aae69afdec8f4e1e8c6ca")
(setf (get vision::ObservationResponse :datatype-) "vision/ObservationResponse")
(setf (get vision::ObservationResponse :definition-)
      "int8 task
---
float32 x
float32 y

")



(provide :vision/Observation "8a3ea109914aae69afdec8f4e1e8c6ca")


