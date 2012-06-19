
(cl:in-package :asdf)

(defsystem "SubImageRecognition-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SwitchAlgorithm" :depends-on ("_package_SwitchAlgorithm"))
    (:file "_package_SwitchAlgorithm" :depends-on ("_package"))
    (:file "ListAlgorithms" :depends-on ("_package_ListAlgorithms"))
    (:file "_package_ListAlgorithms" :depends-on ("_package"))
  ))