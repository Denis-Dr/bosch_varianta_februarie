import SerialHandler
import cv2
import time
import numpy as np
from DetectieBanda import Banda
import DeseneazaBanda
from Starile import DeplasareMasina
# from StopAndPark import stopOrPark
from PIDcontroller import PID

stare = DeplasareMasina()
pid = PID(0.15, 0, 0.1)
target_error = 0
viteza_pwm = 0.19

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)  # ('cameraE.avi')
# cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
cap.set(3, 640)
cap.set(4, 480)

THRESHOLD = 180
__ESTE_PE_MASINA__ = False  # <<-----
VIDEO_RECORD = False

if __ESTE_PE_MASINA__:
    serialHandler = SerialHandler.SerialHandler("/dev/ttyACM0")
    serialHandler.startReadThread()

if VIDEO_RECORD:
    fourcc = cv2.VideoWriter_fourcc(*'DIVX')
    out = cv2.VideoWriter('cameraJ.avi', fourcc, 20, (640, 480))

DEBUG_ALL_DATA = False
AMPARCAT = False
__PRINT_DATE__ = False
__AFISARE_VIDEO__ = True

## VARIABILE
latimeSus = np.zeros(0)
latimeJos = np.zeros(0)
vectorLatimiMedii = np.array([-1, -1])
distantaFataDeAx = 0
centruRelativ = 0
exceptieDeInceput0 = 3  # exceptam primele 3 cadre de la regula de calcul a vectorLatimiMedii pt a obt o latime media reala pe care sa incercam sa o pastram
exceptieDeInceput1 = 3

EroareCentrare = 22

pasAdaptare = 0
pozitieMijlocAnterior = -1
counter = 0
masina = DeplasareMasina()
## END OF VARIABLE

counterStop = 0
contorDistMedBenzi0 = 0  # calculam distanda medie intre benzi
contorDistMedBenzi1 = 0

while True:
    ret, frame = cap.read()
    if ret is False:
        break

    if VIDEO_RECORD:
        out.write(frame)

    points1 = np.float32([[100, 200], [540, 200], [0, 290], [640, 290]])
    points2 = np.float32([[0, 0], [640, 0], [0, 480], [640, 480]])
    P = cv2.getPerspectiveTransform(points1, points2)
    output = cv2.warpPerspective(frame, P, (640, 480))
    img = output

    counter = counter + 1
    if counter < 5:
        continue
    # if VIDEO_RECORD:
    #   out.write(frame)
    # if not __ESTE_PE_MASINA__:
    cv2.putText(img, "Cadrul: " + str(counter), (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                (200, 255, 200), 2)

    gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
    #ret, binarization = cv2.threshold(gray, THRESHOLD, 255, cv2.THRESH_BINARY)
    binarization = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 45, -19)

    inaltimeCadru, lungimeCadru, _ = frame.shape  # H si L imagine
    MijlocCamera = int(lungimeCadru / 2.0)

    Sectiune = Banda()

    inaltimeSectiuneSus = Sectiune.setInaltimeSectiuneSus(int(inaltimeCadru * 0.5))
    inaltimeSectiuneJos = Sectiune.setInaltimeSectiuneJos(int(inaltimeCadru * 0.75))

    centreSectiuni = Sectiune.calculCentreSectiuni(binarization, lungimeCadru)

    ####### calc lat medie banda la fiecare 3 cadre cu detectare ############################
    #########################################################################################

    vectorLatimiBanda = Sectiune.calculLatimeBanda(centreSectiuni)  # calcul latimi banda
    if vectorLatimiBanda == ([-1, -1]):
        vectorLatimiBanda = ([440, 460])
    if contorDistMedBenzi0 <= 3:  # CALCUL LATIME MEDIE BANDA DUPA 3 CADRE CU BANDA DETECTATA
        if ((exceptieDeInceput0 > 0 or (vectorLatimiMedii[0] - 45 < vectorLatimiBanda[0] < vectorLatimiMedii[0] + 45)) and vectorLatimiBanda[0] != -1):
            latimeSus = np.append(latimeSus, vectorLatimiBanda[0])
            contorDistMedBenzi0 += 1
            if exceptieDeInceput0 > 0:
                exceptieDeInceput0 -= 1
    if contorDistMedBenzi0 == 3:
        contorDistMedBenzi0 = 0
        vectorLatimiMedii[0] = int(np.average(latimeSus))
        latimeSus = np.zeros(0)

    if contorDistMedBenzi1 <= 3:  # CALCUL LATIME MEDIE BANDA DUPA 3 CADRE CU BANDA DETECTATA
        if ((exceptieDeInceput1 > 0 or (vectorLatimiMedii[1] - 45 < vectorLatimiBanda[1] < vectorLatimiMedii[1] + 45)) and vectorLatimiBanda[1] != -1):
            latimeJos = np.append(latimeJos, vectorLatimiBanda[1])
            contorDistMedBenzi1 += 1
            if exceptieDeInceput1 > 0:
                exceptieDeInceput1 -= 1
    if contorDistMedBenzi1 == 3:
        contorDistMedBenzi1 = 0
        vectorLatimiMedii[1] = int(np.average(latimeJos))
        latimeJos = np.zeros(0)

    ########################################################################################
    ########################################################################################

    centreSectiuniCompletat = Sectiune.completareCentre(centreSectiuni, vectorLatimiMedii)
    vectorCentreMedii = Sectiune.calculCentreMedii(centreSectiuniCompletat)
    centruRelativ = Sectiune.calculCentruRelativ(vectorCentreMedii)
    distantaFataDeAx = Sectiune.calculDistantaFataDeAx(centruRelativ, MijlocCamera)
    nrBenziDetectate, partea = Sectiune.nrBenziDetectate()

    if __PRINT_DATE__:
        print("\n ----------- FRAME ", counter, " --------------")
        print(" ##centreSectiuni ", centreSectiuni, "\n ##vectorLatimiBanda ", vectorLatimiBanda, "\n ##vectorLatimiMedii ", vectorLatimiMedii, "\n ##centreSectiuniCompletat ",
              centreSectiuniCompletat,
              "\n ##vectorCentreMedii ", vectorCentreMedii, "\n ##distantaFataDeAx ", distantaFataDeAx, "  ##centruRelativ ", centruRelativ, "  ##MijlocCamera", MijlocCamera)

    intersectie = Sectiune.detectareIntersectie(binarization, lungimeCadru)
    # if intersectie == 1:
    # print("--> Urmeaza INTERSECTIE")


    DeseneazaBanda.PutLines(img, binarization, inaltimeCadru, lungimeCadru, inaltimeSectiuneSus, inaltimeSectiuneJos)
    DeseneazaBanda.deseneazaDrum(__PRINT_DATE__, img, centreSectiuniCompletat, centreSectiuni, centruRelativ, distantaFataDeAx, nrBenziDetectate, partea, inaltimeSectiuneSus, inaltimeSectiuneJos,
                            vectorCentreMedii, intersectie, inaltimeCadru, lungimeCadru)  # inFunctiune

    if DEBUG_ALL_DATA and __ESTE_PE_MASINA__:
        print("Benzi gasite:" + str(Sectiune.nrBenziDetectate()))
        print("\nCentre:\t" + str(Sectiune.centreSectiuni))
    else:
        cv2.putText(img, "Benzi identificate: " + str(Sectiune.nrBenziDetectate()), (10, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    try:
        current_error = abs(distantaFataDeAx)
        error = target_error - current_error
        correction = pid.Update(error)
        print("\n ## corr ", correction)

        if correction > 22 and distantaFataDeAx < 0:
            correction = 22
        elif correction > 22 and distantaFataDeAx > 0:
            correction = -22

        if correction < -22 and distantaFataDeAx < 0:
            correction = 22
        elif correction < -22 and distantaFataDeAx > 0:
            correction = -22

        if -22 <= correction <= 0 and distantaFataDeAx < 0:
            correction = -correction
        elif 0 <= correction <= 22 and distantaFataDeAx > 0:
            correction = -correction

        if __ESTE_PE_MASINA__:
            if intersectie == 0:
                serialHandler.sendMove(viteza_pwm, int(correction))
            else:
                stare.on_t_intra_inter(1)

        print("corrention  ", int(correction), "   distAX" ,distantaFataDeAx)

    except Exception as e:
        print(e)
        pass

    '''
    if (not __ESTE_PE_MASINA__) :
        cv2.putText(img, "Stare: " + str(masina.current_state.value), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                    (250, 250, 250), 2)
    else :
        print(masina.current_state.value)
    '''

    if __AFISARE_VIDEO__:
        # cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
        # cv2.resizeWindow('frame', 960, 720)
        #cv2.imshow("frame", frame)

        # cv2.namedWindow('Img_procesata', cv2.WINDOW_NORMAL)
        # cv2.resizeWindow('Img_procesata', 960, 720)
        cv2.imshow("Img_procesata", img)

        # cv2.namedWindow('binarizare', cv2.WINDOW_NORMAL)
        # cv2.resizeWindow('binarizare', 960, 720)
        cv2.imshow("binarizare", binarization)

    key = cv2.waitKey(1)  # 1=readare automata // 0=redare la buton
    if key==ord('q'): #cv2.waitKey(1) & 0xFF == ord('q'):
        if __ESTE_PE_MASINA__:
            serialHandler.sendBrake(0.0)
        break

if __ESTE_PE_MASINA__:
    serialHandler.sendPidActivation(False)
    serialHandler.close()

cap.release()
cv2.destroyAllWindows()
