
(cl:in-package :asdf)

(defsystem "mdb_drone-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "chatter" :depends-on ("_package_chatter"))
    (:file "_package_chatter" :depends-on ("_package"))
    (:file "Obs" :depends-on ("_package_Obs"))
    (:file "_package_Obs" :depends-on ("_package"))
  ))