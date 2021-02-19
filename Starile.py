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
    t_opreste = s_mergi.to(s_oprit)  # opreste la stop
    t_porneste = s_oprit.to(s_mergi)  # porneste de la stop
    t_intra_inter = s_mergi.to(s_inter)
    t_inter_la_dreapta = s_inter.to(s_inter_dreapta)
    t_inter_la_stanga = s_inter.to(s_inter_stanga)
    t_inter_in_fata = s_inter.to(s_inter_fata)
    s_mergiDupaCurba = s_inter_dreapta.to(s_mergi) # ?

    t_iesi_inter_st = s_inter_stanga.to(s_mergi)
    t_iesi_inter_fata = s_inter_fata.to(s_mergi)
    t_iesi_inter_dr = s_inter_dreapta.to(s_mergi)

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
    def on_t_start(self):
        print('Start')
        try:
            global serialHandler
            serialHandler = SerialHandler.SerialHandler("/dev/ttyACM0")
            camera_test = cv2.VideoCapture(0)
        except serialHandler is None:
            print("Eroare ACM")
        except cv2.error:
            print("Eroare Camera")
        finally:
            serialHandler.sendPidActivation(False)
            serialHandler.close()
            #camera_test.release()
            cv2.destroyAllWindows()

        # cautam stopul

    def on_t_opreste(self):
        print('Stop')
        serialHandler.sendBrake(0.0)
        time.sleep(3)
        self.on_t_porneste(self)

    def on_t_porneste(self):
        print('GO GO GO!')

    def on_t_intra_inter(self, directie):
        print("In intersectie")
        '''
        directie = 1  stanga
        directie = 2  in fata
        directie = 3  dreapta
        '''
        if directie == 1:
            self.on_t_inter_la_stanga(self)
        elif directie == 2:
            self.on_t_inter_in_fata(self)
        elif directie == 3:
            self.ont_inter_la_dreapta(self)

    def on_t_inter_la_stanga(self):
        serialHandler.sendMove(0.19, -17)
        time.sleep(7)
        self.on_t_iesi_inter_st(self)

    def on_t_inter_in_fata(self):
        serialHandler.sendMove(0.18, 0)
        time.sleep(1.2)

    def on_t_inter_la_dreapta(self):
        serialHandler.sendMove(0.19, 22)
        time.sleep(1)

    def on_t_iesi_inter_st(self):
        print("A facut la stanga")

    def on_t_iesi_inter_fata(self):
        print("A mers in fata")

    def on_t_iesi_inter_dr(self):
        print("A faccut la dreapta")



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
