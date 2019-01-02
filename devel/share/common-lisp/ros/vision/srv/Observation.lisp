; Auto-generated. Do not edit!


(cl:in-package vision-srv)


;//! \htmlinclude Observation-request.msg.html

(cl:defclass <Observation-request> (roslisp-msg-protocol:ros-message)
  ((task
    :reader task
    :initarg :task
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Observation-request (<Observation-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Observation-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Observation-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vision-srv:<Observation-request> is deprecated: use vision-srv:Observation-request instead.")))

(cl:ensure-generic-function 'task-val :lambda-list '(m))
(cl:defmethod task-val ((m <Observation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-srv:task-val is deprecated.  Use vision-srv:task instead.")
  (task m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Observation-request>) ostream)
  "Serializes a message object of type '<Observation-request>"
  (cl:let* ((signed (cl:slot-value msg 'task)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Observation-request>) istream)
  "Deserializes a message object of type '<Observation-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'task) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Observation-request>)))
  "Returns string type for a service object of type '<Observation-request>"
  "vision/ObservationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Observation-request)))
  "Returns string type for a service object of type 'Observation-request"
  "vision/ObservationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Observation-request>)))
  "Returns md5sum for a message object of type '<Observation-request>"
  "8a3ea109914aae69afdec8f4e1e8c6ca")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Observation-request)))
  "Returns md5sum for a message object of type 'Observation-request"
  "8a3ea109914aae69afdec8f4e1e8c6ca")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Observation-request>)))
  "Returns full string definition for message of type '<Observation-request>"
  (cl:format cl:nil "int8 task~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Observation-request)))
  "Returns full string definition for message of type 'Observation-request"
  (cl:format cl:nil "int8 task~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Observation-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Observation-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Observation-request
    (cl:cons ':task (task msg))
))
;//! \htmlinclude Observation-response.msg.html

(cl:defclass <Observation-response> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0))
)

(cl:defclass Observation-response (<Observation-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Observation-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Observation-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vision-srv:<Observation-response> is deprecated: use vision-srv:Observation-response instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <Observation-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-srv:x-val is deprecated.  Use vision-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <Observation-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-srv:y-val is deprecated.  Use vision-srv:y instead.")
  (y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Observation-response>) ostream)
  "Serializes a message object of type '<Observation-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Observation-response>) istream)
  "Deserializes a message object of type '<Observation-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Observation-response>)))
  "Returns string type for a service object of type '<Observation-response>"
  "vision/ObservationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Observation-response)))
  "Returns string type for a service object of type 'Observation-response"
  "vision/ObservationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Observation-response>)))
  "Returns md5sum for a message object of type '<Observation-response>"
  "8a3ea109914aae69afdec8f4e1e8c6ca")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Observation-response)))
  "Returns md5sum for a message object of type 'Observation-response"
  "8a3ea109914aae69afdec8f4e1e8c6ca")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Observation-response>)))
  "Returns full string definition for message of type '<Observation-response>"
  (cl:format cl:nil "float32 x~%float32 y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Observation-response)))
  "Returns full string definition for message of type 'Observation-response"
  (cl:format cl:nil "float32 x~%float32 y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Observation-response>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Observation-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Observation-response
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Observation)))
  'Observation-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Observation)))
  'Observation-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Observation)))
  "Returns string type for a service object of type '<Observation>"
  "vision/Observation")