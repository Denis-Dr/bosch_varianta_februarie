import cv2


def deseneazaDrum(PRINT_DATE, img, centreSectiuniCompletat, centreSectiuni, centruRelativ, distantaFataDeAx, nrBenziDetectate, partea, inaltimeSectiuneSus, inaltimeSectiuneJos, vectorCentreMedii,
                  intersectie, inaltimeCadru, lungimeCadru ): #inFunctiune
    cv2.putText(img, "Benzi gasite: " + str(nrBenziDetectate), (10, 430), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (140, 140, 210), 2)
    #if (inFunctiune == True):
    cv2.putText(img, "RUNNING", (180, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 250, 0), 2)


    if centruRelativ != 0:
        cv2.arrowedLine(img, (int(lungimeCadru / 2), 180), (int(centruRelativ), 180), (255, 255, 125), 2)
        cv2.putText(img, "Dist: " + str(distantaFataDeAx), (int(lungimeCadru / 2 + 50), 180), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (60, 250, 250), 1)
        cv2.line(img, (int(centruRelativ), 0), (int(centruRelativ), inaltimeCadru), (255, 125, 125), 5)

    if vectorCentreMedii[0] != -1 and vectorCentreMedii[1] != -1:
        cv2.circle(img, (vectorCentreMedii[0], inaltimeSectiuneSus), 3, (200, 0, 230), 3)
        cv2.circle(img, (vectorCentreMedii[1], inaltimeSectiuneJos), 3, (200, 0, 230), 3)
        cv2.line(img, (vectorCentreMedii[0], inaltimeSectiuneSus), (vectorCentreMedii[1], inaltimeSectiuneJos), (170, 0, 0), 2)

    if nrBenziDetectate == 2:
        for j in range(2):
            if centreSectiuni[0][j] != -1:
                cv2.putText(img, str(centreSectiuniCompletat[0][j]), (centreSectiuniCompletat[0][j] - 20, inaltimeSectiuneSus - 8), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 210, 0), 2)
                cv2.circle(img, (centreSectiuniCompletat[0][j], inaltimeSectiuneSus), 3, (0, 0, 200), 3)
            else:
                cv2.putText(img, str(centreSectiuniCompletat[0][j]), (centreSectiuniCompletat[0][j] - 20, inaltimeSectiuneSus - 8), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 30, 200), 2)
                cv2.circle(img, (centreSectiuniCompletat[0][j], inaltimeSectiuneSus), 3, (200, 0, 0), 3)

        for j in range(2):
            if centreSectiuni[1][j] != -1:
                cv2.putText(img, str(centreSectiuniCompletat[1][j]), (centreSectiuniCompletat[1][j] - 20, inaltimeSectiuneJos - 8), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 210, 0), 2)
                cv2.circle(img, (centreSectiuniCompletat[1][j], inaltimeSectiuneJos), 3, (0, 0, 200), 3)
            else:
                cv2.putText(img, str(centreSectiuniCompletat[1][j]), (centreSectiuniCompletat[1][j] - 20, inaltimeSectiuneJos - 8), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 30, 200), 2)
                cv2.circle(img, (centreSectiuniCompletat[1][j], inaltimeSectiuneJos), 3, (200, 0, 0), 3)

    elif nrBenziDetectate == 1:
        if partea == "stanga":
            cv2.putText(img, "Detecteaza banda stanga", (380, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 190), 2)
            if PRINT_DATE:
                print("Detecteaza banda stanga. Pozitia aprox. a benzii dreapta este: ", centreSectiuniCompletat[0][1], "; ", centreSectiuniCompletat[1][1])
            if centreSectiuniCompletat[0][0] != -1:
                cv2.putText(img, str(centreSectiuniCompletat[0][0]), (centreSectiuniCompletat[0][0] - 20, inaltimeSectiuneSus - 8), cv2.FONT_HERSHEY_SIMPLEX, 1, (200, 30, 0), 2)
                cv2.circle(img, (centreSectiuniCompletat[0][0], inaltimeSectiuneSus), 3, (0, 200, 0), 3)

            if centreSectiuniCompletat[1][0] != -1:
                cv2.putText(img, str(centreSectiuniCompletat[1][0]), (centreSectiuniCompletat[1][0] - 20, inaltimeSectiuneJos - 8), cv2.FONT_HERSHEY_SIMPLEX, 1, (200, 30, 0), 2)
                cv2.circle(img, (centreSectiuniCompletat[1][0], inaltimeSectiuneJos), 3, (0, 200, 0), 3)

        elif partea == "dreapta":
            cv2.putText(img, "Detecteaza banda dreapta", (380, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 190), 2)
            if PRINT_DATE:
                print("Detecteaza banda dreapta. Pozitia aprox. a benzii stanga este: ", centreSectiuniCompletat[0][0], "; ", centreSectiuniCompletat[1][0])
            if centreSectiuniCompletat[0][1] != -1:
                cv2.putText(img, str(centreSectiuniCompletat[0][1]), (centreSectiuniCompletat[0][1] - 20, inaltimeSectiuneSus - 8), cv2.FONT_HERSHEY_SIMPLEX, 1, (200, 30, 0), 2)
                cv2.circle(img, (centreSectiuniCompletat[0][1], inaltimeSectiuneSus), 3, (0, 200, 0), 3)

            if centreSectiuniCompletat[1][1] != -1:
                cv2.putText(img, str(centreSectiuniCompletat[1][1]), (centreSectiuniCompletat[1][1] - 20, inaltimeSectiuneJos - 8), cv2.FONT_HERSHEY_SIMPLEX, 1, (200, 30, 0), 2)
                cv2.circle(img, (centreSectiuniCompletat[1][1], inaltimeSectiuneJos), 3, (0, 200, 0), 3)
    else:
        if PRINT_DATE:
            print("Nicio banda detectata!!!")

    if intersectie == 1:
        cv2.putText(img, "URMEAZA INTERSECTIE", (360, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 90, 255), 2)

def PutLines(img, binarization, inaltimeCadru, lungimeCadru, inaltimeSectiuneSus, inaltimeSectiuneJos):

    cv2.line(img, (int(0.02 * lungimeCadru), int(inaltimeCadru * 0.5)), (int(0.46 * lungimeCadru), int(inaltimeCadru * 0.5)), (255, 255, 0), 2)
    cv2.line(img, (int(0.54 * lungimeCadru), inaltimeSectiuneSus), (int(0.98 * lungimeCadru), inaltimeSectiuneSus), (255, 255, 0), 2)

    cv2.line(img, (0, inaltimeSectiuneJos), (lungimeCadru, inaltimeSectiuneJos), (255, 255, 0), 2)

    cv2.line(img, (int(lungimeCadru / 2), 0), (int(lungimeCadru / 2), inaltimeCadru), (255, 255, 255), 2)  # linia verticala

    cv2.line(binarization, (int(0.02 * lungimeCadru), inaltimeSectiuneSus), (int(0.46 * lungimeCadru), inaltimeSectiuneSus), (255, 255, 0), 2)
    cv2.line(binarization, (int(0.54 * lungimeCadru), inaltimeSectiuneSus), (int(0.98 * lungimeCadru), inaltimeSectiuneSus), (255, 255, 0), 2)

    cv2.line(binarization, (0, inaltimeSectiuneJos), (lungimeCadru, inaltimeSectiuneJos), (255, 255, 0), 2)
    cv2.line(binarization, (int(lungimeCadru / 2), 0), (int(lungimeCadru / 2), inaltimeCadru), (255, 255, 255), 2)  # linia verticala

    cv2.line(img, (int(0.33 * lungimeCadru), inaltimeSectiuneSus), (int(0.33 * lungimeCadru), inaltimeSectiuneJos), (255, 0, 255), 2)
    cv2.line(img, (int(0.5 * lungimeCadru), inaltimeSectiuneSus), (int(0.5 * lungimeCadru), inaltimeSectiuneJos), (255, 0, 255), 2)
    cv2.line(img, (int(0.67 * lungimeCadru), inaltimeSectiuneSus), (int(0.67 * lungimeCadru), inaltimeSectiuneJos), (255, 0, 255), 2)

