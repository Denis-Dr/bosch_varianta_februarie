from statemachine import StateMachine, State
import cv2
import SerialHandler
import time

global serialHandler


class DeplasareMasina(StateMachine):
    # STARILE
    s_initiala = State('Initializare', initial=True)
    s_mergi = State('Merge inainte')
    s_oprit = State('Oprit')
    s_inter = State('Intersectie')
    s_inter_stanga = State('La stanga in intersectie')
    s_inter_dreapta = State('La dreapta in intersectie')
    s_inter_fata = State('In fata in intersectie')
    s_parcatL = State('ParcheazaLaterala')
    #CurbaStangaDupaStopActiune = State('CurbaStangaDupaStop')

    # TRANZITIILE
    t_start = s_initiala.to(s_mergi)
    t_opreste = s_mergi.to(s_oprit)
    t_porneste = s_oprit.to(s_mergi)
    t_intra_inter = s_mergi.to(s_inter)
    t_inter_la_dreapta = s_inter.to(s_inter_dreapta)
    t_inter_la_stanga = s_inter.to(s_inter_stanga)
    t_inter_la_fata = s_inter.to(s_inter_fata)
    s_mergiDupaCurba = s_inter_dreapta.to(s_mergi)

    Parcheaza = s_mergi.to(s_parcatL)

    # STARILE
    def on_s_initiala(self):
        print("Initializare...")

    def on_s_mergi(self):
        print("Merge inainte")

    def on_s_oprit(self):
        print("Oprit")
        time.sleep(2)

    def on_s_inter_dreapta(self):
        print("La dreapta in intersectie")

    def on_CurbaStanga(self):
        print("Curba la stanga")


    def on_s_parcatL(self):
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
