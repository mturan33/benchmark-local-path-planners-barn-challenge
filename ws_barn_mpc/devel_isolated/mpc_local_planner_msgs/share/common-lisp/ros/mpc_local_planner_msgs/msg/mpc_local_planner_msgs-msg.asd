
(cl:in-package :asdf)

(defsystem "mpc_local_planner_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "OptimalControlResult" :depends-on ("_package_OptimalControlResult"))
    (:file "_package_OptimalControlResult" :depends-on ("_package"))
    (:file "StateFeedback" :depends-on ("_package_StateFeedback"))
    (:file "_package_StateFeedback" :depends-on ("_package"))
  ))