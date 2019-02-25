;;;
;;; Copyright (c) 2018, Mihai Pomarlan <pomarlan@uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of the Institute for Artificial Intelligence/
;;;       Universitaet Bremen nor the names of its contributors may be used to
;;;       endorse or promote products derived from this software without
;;;       specific prior written permission.
;;;
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :skram)

(prolog:def-fact-group skram-actions (desig:action-grounding)

  (prolog:<- (desig:action-grounding ?action-designator (infiltrate ?target-designator))
    (spec:property ?action-designator (:type :infiltrate))
    (prolog:lisp-fun desig:desig-prop-value ?action-designator :target ?target-designator))

  (prolog:<- (desig:action-grounding ?action-designator (schematic-transport ?actee-designator ?source-designator ?destination-designator ?instrument-designator))
    (spec:property ?action-designator (:type :schematic-transport))
    (prolog:lisp-fun desig:desig-prop-value ?action-designator :objects ?actee-designator)
    (prolog:lisp-fun desig:desig-prop-value ?action-designator :source ?source-designator)
    (prolog:lisp-fun desig:desig-prop-value ?action-designator :instrument ?instrument-designator)
    (prolog:lisp-fun desig:desig-prop-value ?action-designator :destination ?destination-designator))

  (prolog:<- (desig:action-grounding ?action-designator (schematic-ingesting ?actee-designator ?source-designator ?instrument-designator))
    (spec:property ?action-designator (:type :ingesting))
    (prolog:lisp-fun desig:desig-prop-value ?action-designator :objects ?actee-designator)
    (prolog:lisp-fun desig:desig-prop-value ?action-designator :source ?source-designator)
    (prolog:lisp-fun desig:desig-prop-value ?action-designator :instrument ?instrument-designator))

  (prolog:<- (desig:action-grounding ?action-designator (schematic-material-removal ?material-designator ?support-designator ?source-designator ?material-destination-designator ?support-destination-designator ?instrument-designator))
    (spec:property ?action-designator (:type :material-removal))
    (prolog:lisp-fun desig:desig-prop-value ?action-designator :material ?material-designator)
    (prolog:lisp-fun desig:desig-prop-value ?action-designator :objects ?support-designator)
    (prolog:lisp-fun desig:desig-prop-value ?action-designator :source ?source-designator)
    (prolog:lisp-fun desig:desig-prop-value ?action-designator :instrument ?instrument-designator)
    (prolog:lisp-fun desig:desig-prop-value ?action-designator :support-destination ?support-destination-designator)
    (prolog:lisp-fun desig:desig-prop-value ?action-designator :material-destination ?material-destination-designator))
  )

