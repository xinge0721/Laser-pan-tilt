;; Auto-generated. Do not edit!


(when (boundp 'Serial::date)
  (if (not (find-package "SERIAL"))
    (make-package "SERIAL"))
  (shadow 'date (find-package "SERIAL")))
(unless (find-package "SERIAL::DATE")
  (make-package "SERIAL::DATE"))
(unless (find-package "SERIAL::DATEREQUEST")
  (make-package "SERIAL::DATEREQUEST"))
(unless (find-package "SERIAL::DATERESPONSE")
  (make-package "SERIAL::DATERESPONSE"))

(in-package "ROS")





(defclass Serial::dateRequest
  :super ros::object
  :slots (_mood ))

(defmethod Serial::dateRequest
  (:init
   (&key
    ((:mood __mood) 0)
    )
   (send-super :init)
   (setq _mood (round __mood))
   self)
  (:mood
   (&optional __mood)
   (if __mood (setq _mood __mood)) _mood)
  (:serialization-length
   ()
   (+
    ;; int32 _mood
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _mood
       (write-long _mood s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _mood
     (setq _mood (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass Serial::dateResponse
  :super ros::object
  :slots (_x _y ))

(defmethod Serial::dateResponse
  (:init
   (&key
    ((:x __x) 0)
    ((:y __y) 0)
    )
   (send-super :init)
   (setq _x (round __x))
   (setq _y (round __y))
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
    ;; int32 _x
    4
    ;; int32 _y
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _x
       (write-long _x s)
     ;; int32 _y
       (write-long _y s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _x
     (setq _x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _y
     (setq _y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass Serial::date
  :super ros::object
  :slots ())

(setf (get Serial::date :md5sum-) "bd888d811c7cb3be0c1173dc8d4d390b")
(setf (get Serial::date :datatype-) "Serial/date")
(setf (get Serial::date :request) Serial::dateRequest)
(setf (get Serial::date :response) Serial::dateResponse)

(defmethod Serial::dateRequest
  (:response () (instance Serial::dateResponse :init)))

(setf (get Serial::dateRequest :md5sum-) "bd888d811c7cb3be0c1173dc8d4d390b")
(setf (get Serial::dateRequest :datatype-) "Serial/dateRequest")
(setf (get Serial::dateRequest :definition-)
      "int32 mood
---
int32 x
int32 y
")

(setf (get Serial::dateResponse :md5sum-) "bd888d811c7cb3be0c1173dc8d4d390b")
(setf (get Serial::dateResponse :datatype-) "Serial/dateResponse")
(setf (get Serial::dateResponse :definition-)
      "int32 mood
---
int32 x
int32 y
")



(provide :Serial/date "bd888d811c7cb3be0c1173dc8d4d390b")


