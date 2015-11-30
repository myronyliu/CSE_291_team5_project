
(cl:in-package :asdf)

(defsystem "CSE_291_team5_project-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "CVMessage" :depends-on ("_package_CVMessage"))
    (:file "_package_CVMessage" :depends-on ("_package"))
  ))