from statemachine import StateMachine, State
import cv2
import SerialHandler
import time

global serialHandler


class DeplasareMasina(StateMachine):
    # STARILE
    Initializare = State('Initializare', initial=True)
    MergiInainte = State('MergiInainte')
    Oprit = State('Oprit')
    CurbaStanga = State('IaCurbaStanga')
    CurbaDreapta = State('IaCurbaDreapta')
    ParcareLaterala = State('ParcheazaLaterala')
    PlecareDinParcare = State('PleacaDinParcare')  # asta nu e tranzitie?
    CurbaStangaDupaStopActiune = State('CurbaStangaDupaStop')

    # TRANZITIILE
    PleacaDeLaStart = Initializare.to(MergiInainte)
    Opreste = MergiInainte.to(Oprit)
    stoptodo = Initializare.to(Oprit)   # pt ce?
    PleacaDeLaStop = Oprit.to(MergiInainte)
    CurbaStangaDupaStop = Oprit.to(CurbaStangaDupaStopActiune)
    MergiInainteDupaStop = CurbaStangaDupaStopActiune.to(MergiInainte)
    MergiLaDreapta = MergiInainte.to(CurbaDreapta)
    MergiInainteDupaCurba = CurbaDreapta.to(MergiInainte)
### da
    Parcheaza = MergiInainte.to(ParcareLaterala)
    PleacaDinParcare = ParcareLaterala.to(PlecareDinParcare)
    MergiInainteDupaParcare = PlecareDinParcare.to(MergiInainte)

    # STARILE
    def on_Initializare(self):
        print("Initializare...")

    def on_MergiInainte(self):
        print("Merge inainte")

    def on_Oprit(self):
        print("Oprit")
        time.sleep(2)

    def on_CurbaDreapta(self):
        print("Cruba la dreapta")

    def on_CurbaStanga(self):
        print("Curba la stanga")

    def on_ParcareLaterala(self):
        print("Parcata lateral")
        time.sleep(3)

    def on_PlecareDinParcare(self):
        print("Pleaca din parcare")

    def on_CurbaStangaDupaStopActiune(self):
        print("Curba stanga")


    # TRANZITIILE
    def on_PleacaDeLaStart(self):
        print('Start')
        try:
            global serialHandler
            serialHandler = SerialHandler.SerialHandler("/dev/ttyACM0")
            camera_test = cv2.VideoCapture(0)
        except serialHandler is None:
            print("Eroare ACM")
        except cv2.error:
            print("Eroare Camera")

        # cautam stopul

    def on_Opreste(self):
        print('Stop')
        time.sleep(2.5)

    def on_PleacaDeLaStop(self):
        print('GO GO GO!')
        # cautam Drumul

    def on_MergiLaDreapta(self):
        print('o luam la dreapta - sendMove()')
        # cautam Drumul

    def on_Parcheaza(self):

        serialHandler.sendPidActivation(True)
        serialHandler.sendMove(-0.2, 22.0)
        time.sleep(1)
        serialHandler.sendMove(-0.2, -22.0)
        time.sleep(1)
        serialHandler.sendBrake(0.0)
        time.sleep(2.5)

    def on_PleacaDinParcare(self):
        serialHandler = SerialHandler.SerialHandler("/dev/ttyACM0")
        serialHandler.sendMove(0.2, -22.0)
        time.sleep(1)
        serialHandler.sendMove(0.2, 22.0)
        time.sleep(1)
