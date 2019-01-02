
(cl:in-package :asdf)

(defsystem "vision-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Observation" :depends-on ("_package_Observation"))
    (:file "_package_Observation" :depends-on ("_package"))
  ))