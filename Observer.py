from statemachine import StateMachine, State
import serial
import cv2
# import SerialHandler
import time
# import statemachine
# https://pypi.org/project/python-statemachine/


class DeplasareMasina(StateMachine):
    # STARILE
    Initializare = State('Initializare', initial=True)
    MergiInainte = State('MergiInainte')
    Oprit = State('Oprit')
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

    Parcheaza = MergiInainte.to(ParcareLaterala)  # TODO: ar trebui in loc de Initializare ceva de genu MergInainteDupaU
    PleacaDinParcare = ParcareLaterala.to(PlecareDinParcare)
    MergiInainteDupaParcare = PlecareDinParcare.to(MergiInainte)

    # STARILE
    def on_Initializare(self):
        print("Initializare...")
        try:
            print("da")
        except:
            print("nu")
    # TODO: pt a verifica ce dispozitiv e conectat
    #   trebuie creat obiect pt fiecare cu serial.Serial?,// dupa folosim del//??

    # TRANZITIILE
    def on_PleacaDeLaStart(self):
        print('Hai ca plecam')
        # cautam stopul

    def on_Opreste(self):
        print('STOP.')
        time.sleep(2.5)

    def on_PleacaDeLaStop(self):
        print('GO GO GO!')
        # cautam Drumul
    def on_MergiLaDreapta(self):
        print('o luam la dreapta - sendMove()')
        # cautam Drumul

    def on_Parcheaza(self):
        global serialHandler
        serialHandler = SerialHandler.SerialHandler("/dev/ttyACM0")
        try:
            ## PARCARE STARE
            serialHandler.sendPidActivation(True)
            serialHandler.sendMove(-0.2, 22.0)
            time.sleep(2.7)
            serialHandler.sendMove(-0.2, -22.0)
            time.sleep(2.5)
            serialHandler.sendBrake(0.0)
            time.sleep(2.5)
            ### END OF PARCARE

            ### START PLECARE DIN PARCARE
            serialHandler.sendMove(0.2, -22.0)
            time.sleep(2.2)
            serialHandler.sendMove(0.2, 22.0)
            time.sleep(2.7)
        except:
            print("wtf - n")

# cautam stopul
# masina.stop()
# masina.PleacaDeLaStop()
# masina.
