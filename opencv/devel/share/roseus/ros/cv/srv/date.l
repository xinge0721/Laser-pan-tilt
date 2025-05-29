;; Auto-generated. Do not edit!


(when (boundp 'cv::date)
  (if (not (find-package "CV"))
    (make-package "CV"))
  (shadow 'date (find-package "CV")))
(unless (find-package "CV::DATE")
  (make-package "CV::DATE"))
(unless (find-package "CV::DATEREQUEST")
  (make-package "CV::DATEREQUEST"))
(unless (find-package "CV::DATERESPONSE")
  (make-package "CV::DATERESPONSE"))

(in-package "ROS")





(defclass cv::dateRequest
  :super ros::object
  :slots (_mood ))

(defmethod cv::dateRequest
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

(defclass cv::dateResponse
  :super ros::object
  :slots (_x _y ))

(defmethod cv::dateResponse
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

(defclass cv::date
  :super ros::object
  :slots ())

(setf (get cv::date :md5sum-) "bd888d811c7cb3be0c1173dc8d4d390b")
(setf (get cv::date :datatype-) "cv/date")
(setf (get cv::date :request) cv::dateRequest)
(setf (get cv::date :response) cv::dateResponse)

(defmethod cv::dateRequest
  (:response () (instance cv::dateResponse :init)))

(setf (get cv::dateRequest :md5sum-) "bd888d811c7cb3be0c1173dc8d4d390b")
(setf (get cv::dateRequest :datatype-) "cv/dateRequest")
(setf (get cv::dateRequest :definition-)
      "int32 mood
---
int32 x
int32 y
")

(setf (get cv::dateResponse :md5sum-) "bd888d811c7cb3be0c1173dc8d4d390b")
(setf (get cv::dateResponse :datatype-) "cv/dateResponse")
(setf (get cv::dateResponse :definition-)
      "int32 mood
---
int32 x
int32 y
")



(provide :cv/date "bd888d811c7cb3be0c1173dc8d4d390b")


