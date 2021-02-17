import numpy as np
import Constante as C
from threading import *

''' 
    Modulul detecteaza benzile prin intermediul a 6 sectini si calculeaza centrele pentru fiecare + centrul relativ al benzii (!de implementat).
    Functia calculCentreSectiuni() primeste imaginea binarizata (treshold) si lungimea cadrului (orizontala) si returneaza o 
matrice 2x2 cu abscisele centrelor. Inaltimile centrelor sunt prestabilite in fisierul principal.
    Matricea furnizeaza doua informatii: centrele si ne spune daca se detecteaaza banda (elemente != C.NU_AM_GASIT).
'''


class Banda:
    def __init__(self):
        self.centreSectiuni = np.array([[C.NU_AM_GASIT, C.NU_AM_GASIT], [C.NU_AM_GASIT, C.NU_AM_GASIT]])  # resetam centreSectiuni[][] dupa fiecare cadru
        self.vectorCentreMedii = np.array([C.NU_AM_GASIT, C.NU_AM_GASIT])
        self.inaltimeSectiuneSus = 0
        self.inaltimeSectiuneJos = 0
        self.centruRelativ = 0
        self.distantaFataDeAx = 0

    def setInaltimeSectiuneSus(self, valoare):
        self.inaltimeSectiuneSus = int(valoare)
        return self.inaltimeSectiuneSus

    def setInaltimeSectiuneJos(self, valoare):
        self.inaltimeSectiuneJos = int(valoare)
        return self.inaltimeSectiuneJos

    def calculCentreSectiuni(self, binarization, lungimeCadru):  # calculul centrelor celor 6 sectiuni

        def f1():
            self.contorLungime = 0
            self.incepeSectiunea = 0  # sectiune sus stanga
            for i in range(int(lungimeCadru * 0.02), int(lungimeCadru * 0.46)):
                if binarization[self.inaltimeSectiuneSus, i] == 255:
                    if self.incepeSectiunea == 0:
                        self.incepeSectiunea = 1
                        self.inceput = i
                    self.contorLungime += 1

                if self.incepeSectiunea == 1 and (binarization[self.inaltimeSectiuneSus, i] == 0 or i == (int(lungimeCadru * 0.46) - 1)):
                    self.incepeSectiunea == 0
                    self.sfarsit = i - 1
                    if self.contorLungime < C.EROARE_GROSIME_BANDA and (self.contorLungime > C.EROARE_ARTEFACT or
                                                                        int(lungimeCadru * 0.02) + C.EROARE_ARTEFACT < self.sfarsit < int(lungimeCadru * 0.02) - C.EROARE_ARTEFACT):
                        # eliminam eroarea = sectiune pream mare
                        self.mijlocSectiune = int((self.sfarsit + self.inceput) / 2)
                        self.centreSectiuni[0][0] = self.mijlocSectiune  # atribuim centrul calculat sectiunii corespunzatoare
                        break
                    else:
                        break

        def f2():
            self.contorLungime = 0
            self.incepeSectiunea = 0  # sectiune sus dreapta
            for i in range(int(lungimeCadru * 0.98), int(lungimeCadru * 0.54), - 1):
                if binarization[self.inaltimeSectiuneSus, i] == 255:
                    if self.incepeSectiunea == 0:
                        self.incepeSectiunea = 1
                        self.inceput = i
                    self.contorLungime += 1

                if self.incepeSectiunea == 1 and (binarization[self.inaltimeSectiuneSus, i] == 0 or i == (int(lungimeCadru * 0.54) + 1)):
                    self.incepeSectiunea == 0
                    self.sfarsit = i + 1
                    if self.contorLungime < C.EROARE_GROSIME_BANDA and (self.contorLungime > C.EROARE_ARTEFACT or
                                                                        int(lungimeCadru * 0.54) + C.EROARE_ARTEFACT < self.sfarsit < int(lungimeCadru * 0.98) - C.EROARE_ARTEFACT):

                        self.mijlocSectiune = int((self.sfarsit + self.inceput) / 2)
                        self.centreSectiuni[0][1] = self.mijlocSectiune  # atribuim centrul calculat sectiunii corespunzatoare
                        break
                    else:
                        break

        def f3():
            self.contorLungime = 0
            self.incepeSectiunea = 0  # sectiune jos stanga
            for i in range(1, int(lungimeCadru * 0.5)):
                if binarization[self.inaltimeSectiuneJos, i] == 255:
                    if self.incepeSectiunea == 0:
                        self.incepeSectiunea = 1
                        self.inceput = i
                    self.contorLungime += 1

                if self.incepeSectiunea == 1 and (binarization[self.inaltimeSectiuneJos, i] == 0 or i == (int(lungimeCadru * 0.5) - 1)):
                    self.incepeSectiunea == 0
                    self.sfarsit = i - 1
                    if self.contorLungime < C.EROARE_GROSIME_BANDA and (self.contorLungime > C.EROARE_ARTEFACT or
                                                                        1 + C.EROARE_ARTEFACT < self.sfarsit < int(lungimeCadru * 0.5) - C.EROARE_ARTEFACT):

                        self.mijlocSectiune = int((self.sfarsit + self.inceput) / 2)
                        self.centreSectiuni[1][0] = self.mijlocSectiune  # atribuim centrul calculat sectiunii corespunzatoare
                        break
                    else:
                        break
        def f4():
            self.contorLungime = 0
            self.incepeSectiunea = 0  # sectiune jos dreapta
            for i in range(lungimeCadru - 1, int(lungimeCadru * 0.5), - 1):
                if binarization[self.inaltimeSectiuneJos, i] == 255:
                    if self.incepeSectiunea == 0:
                        self.incepeSectiunea = 1
                        self.inceput = i
                    self.contorLungime += 1

                if self.incepeSectiunea == 1 and (binarization[self.inaltimeSectiuneJos, i] == 0 or i == (int(lungimeCadru * 0.5) + 1)):
                    self.incepeSectiunea == 0
                    self.sfarsit = i + 1
                    if self.contorLungime < C.EROARE_GROSIME_BANDA and (self.contorLungime > C.EROARE_ARTEFACT or
                                                                        int(lungimeCadru * 0.5) + C.EROARE_ARTEFACT < self.sfarsit < lungimeCadru - 1 - C.EROARE_ARTEFACT):

                        self.mijlocSectiune = int((self.sfarsit + self.inceput) / 2)
                        self.centreSectiuni[1][1] = self.mijlocSectiune  # atribuim centrul calculat sectiunii corespunzatoare
                        break
                    else:
                        break

        self.t1 = Thread(target=f1)
        self.t2 = Thread(target=f2)
        self.t3 = Thread(target=f3)
        self.t4 = Thread(target=f4)
        self.t1.start()
        self.t2.start()
        self.t3.start()
        self.t4.start()
        self.t1.join()
        self.t2.join()
        self.t3.join()
        self.t4.join()

        if abs(self.centreSectiuni[1][0] - self.centreSectiuni[0][0]) > 150 and (self.centreSectiuni[1][0] != C.NU_AM_GASIT and self.centreSectiuni[0][0] != C.NU_AM_GASIT):
            self.centreSectiuni[0][0] = C.NU_AM_GASIT  # eliminam cazul in care avem reflexia soarelui pe podea (mai ales in curba)
        elif abs(self.centreSectiuni[1][1] - self.centreSectiuni[0][1]) > 150 and (self.centreSectiuni[1][1] != C.NU_AM_GASIT and self.centreSectiuni[0][1] != C.NU_AM_GASIT):
            self.centreSectiuni[0][1] = C.NU_AM_GASIT

        if (self.centreSectiuni[1][0] != C.NU_AM_GASIT and self.centreSectiuni[0][1] != C.NU_AM_GASIT and self.centreSectiuni[0][0] == C.NU_AM_GASIT
                and self.centreSectiuni[1][1] == C.NU_AM_GASIT):  # eliminam cazul cand, in curba, banda e detectata pe partea cealalta
            self.centreSectiuni[0][1] = C.NU_AM_GASIT
        elif (self.centreSectiuni[1][1] != C.NU_AM_GASIT and self.centreSectiuni[0][0] != C.NU_AM_GASIT and self.centreSectiuni[0][1] == C.NU_AM_GASIT
              and self.centreSectiuni[1][0] == C.NU_AM_GASIT):
            self.centreSectiuni[0][0] = C.NU_AM_GASIT

        if abs(self.centreSectiuni[1][1] - self.centreSectiuni[1][0]) < 50 and (self.centreSectiuni[1][1] != C.NU_AM_GASIT and self.centreSectiuni[1][0] != C.NU_AM_GASIT):
            # eliminam cazul cand una din benzi este detectata pe partea cealalta
            if self.centreSectiuni[0][0] == C.NU_AM_GASIT and self.centreSectiuni[0][1] != C.NU_AM_GASIT:
                self.centreSectiuni[0][1] = C.NU_AM_GASIT
                self.centreSectiuni[1][1] = C.NU_AM_GASIT
            elif self.centreSectiuni[0][0] != C.NU_AM_GASIT and self.centreSectiuni[0][1] == C.NU_AM_GASIT:
                self.centreSectiuni[0][0] = C.NU_AM_GASIT
                self.centreSectiuni[1][0] = C.NU_AM_GASIT

        return self.centreSectiuni  # returnam matricea cu centrele sectiunilor

    def calculCentreMedii(self, centreSectiuniCompletat):
        if centreSectiuniCompletat[0][0] != C.NU_AM_GASIT and centreSectiuniCompletat[0][1] != C.NU_AM_GASIT:  # calcul centru mediu sus
            self.vectorCentreMedii[0] = int((centreSectiuniCompletat[0][0] + centreSectiuniCompletat[0][1]) / 2)
        if centreSectiuniCompletat[1][0] != C.NU_AM_GASIT and centreSectiuniCompletat[1][1] != C.NU_AM_GASIT:  # calcul centru mediu jos
            self.vectorCentreMedii[1] = int((centreSectiuniCompletat[1][0] + centreSectiuniCompletat[1][1]) / 2)
        return self.vectorCentreMedii

    def calculCentruRelativ(self, vectorCentreMedii):
        self.contor = 0
        self.suma = 0

        if self.vectorCentreMedii[0] != C.NU_AM_GASIT:
            self.suma = self.suma + vectorCentreMedii[0]
            self.contor = self.contor + 1

        if self.vectorCentreMedii[1] != C.NU_AM_GASIT:
            self.suma = self.suma + vectorCentreMedii[1]
            self.contor = self.contor + 1

        if self.contor != 0:
            self.centruRelativ = int(self.suma / self.contor)

        return self.centruRelativ

    def calculDistantaFataDeAx(self, centruRelativ, MijlocCamera):  # calc dist dintre centru camera si centru banda
        self.distantaFataDeAx = MijlocCamera - centruRelativ

        return self.distantaFataDeAx

    def nrBenziDetectate(self):  # calc nr benzi / partea pe care detect
        if ((self.centreSectiuni[0][0] == C.NU_AM_GASIT and self.centreSectiuni[1][0] == C.NU_AM_GASIT) and
                (self.centreSectiuni[0][1] != C.NU_AM_GASIT or self.centreSectiuni[1][1] != C.NU_AM_GASIT)):
            return 1, "dreapta"

        elif ((self.centreSectiuni[0][0] != C.NU_AM_GASIT or self.centreSectiuni[1][0] != C.NU_AM_GASIT) and
              (self.centreSectiuni[0][1] == C.NU_AM_GASIT and self.centreSectiuni[1][1] == C.NU_AM_GASIT)):
            return 1, "stanga"

        elif ((self.centreSectiuni[0][0] != C.NU_AM_GASIT or self.centreSectiuni[1][0] != C.NU_AM_GASIT) and
              (self.centreSectiuni[0][1] != C.NU_AM_GASIT or self.centreSectiuni[1][1] != C.NU_AM_GASIT)):
            return 2, ""

        else:
            return 0, ""

    def detectareIntersectie(self, binarization, lungimeCadru):  # detecteaza linia orizontala de la inceputul intersectiei
        self.vectorIntersectie = [C.NU_AM_GASIT, C.NU_AM_GASIT, C.NU_AM_GASIT]
        self.contorLungime1 = 0
        self.contorLungime2 = 0
        self.contorLungime3 = 0
        self.incepeSectiunea1 = 0
        self.incepeSectiunea2 = 0
        self.incepeSectiunea3 = 0

        for i in range(self.inaltimeSectiuneSus, self.inaltimeSectiuneJos):
            if binarization[i, int(0.33 * lungimeCadru)] == 255:  # sectiunea STANGA
                if self.incepeSectiunea1 == 0:
                    self.incepeSectiunea1 = 1
                    self.inceput1 = i
                self.contorLungime1 += 1
            if self.incepeSectiunea1 == 1 and (binarization[i, int(0.33 * lungimeCadru)] == 0 or i == (self.inaltimeSectiuneJos - 1)):
                self.incepeSectiunea1 == 0
                self.sfarsit1 = i - 1
                if self.contorLungime1 < C.EROARE_GROSIME_BANDA:  # eliminam eroarea = sectiune pream mare
                    self.mijloc1 = int((self.sfarsit1 + self.inceput1) / 2)
                    self.vectorIntersectie[0] = self.mijloc1

            if binarization[i, int(0.5 * lungimeCadru)] == 255:  # sectiunea MIJLOC
                if self.incepeSectiunea2 == 0:
                    self.incepeSectiunea2 = 1
                    self.inceput2 = i
                self.contorLungime2 += 1
            if self.incepeSectiunea2 == 1 and (binarization[i, int(0.5 * lungimeCadru)] == 0 or i == (self.inaltimeSectiuneJos - 1)):
                self.incepeSectiunea2 == 0
                self.sfarsit2 = i - 1
                if self.contorLungime2 < C.EROARE_GROSIME_BANDA:  # eliminam eroarea = sectiune pream mare
                    self.mijloc2 = int((self.sfarsit2 + self.inceput2) / 2)
                    self.vectorIntersectie[1] = self.mijloc2

            if binarization[i, int(0.67 * lungimeCadru)] == 255:  # sectiunea DREAPTA
                if self.incepeSectiunea3 == 0:
                    self.incepeSectiunea3 = 1
                    self.inceput3 = i
                self.contorLungime3 += 1
            if self.incepeSectiunea3 == 1 and (binarization[i, int(0.67 * lungimeCadru)] == 0 or i == (self.inaltimeSectiuneJos - 1)):
                self.incepeSectiunea3 == 0
                self.sfarsit3 = i - 1
                if self.contorLungime3 < C.EROARE_GROSIME_BANDA:  # eliminam eroarea = sectiune pream mare
                    self.mijloc3 = int((self.sfarsit3 + self.inceput3) / 2)
                    self.vectorIntersectie[2] = self.mijloc3

        if (self.vectorIntersectie[0] != C.NU_AM_GASIT and self.vectorIntersectie[1] != C.NU_AM_GASIT and self.vectorIntersectie[2] != C.NU_AM_GASIT):
            if (self.vectorIntersectie[0] <= self.vectorIntersectie[1] <= self.vectorIntersectie[2]) or (self.vectorIntersectie[0] >= self.vectorIntersectie[1] >= self.vectorIntersectie[2]):
                return 1
        else:
            return 0

    def calculLatimeBanda(self, centreSectiuni):
        self.vectorLatimiBanda = [C.NU_AM_GASIT, C.NU_AM_GASIT]
        if centreSectiuni[0][1] != C.NU_AM_GASIT and centreSectiuni[0][0] != C.NU_AM_GASIT:
            self.vectorLatimiBanda[0] = centreSectiuni[0][1] - centreSectiuni[0][0]  # latime sus

        if centreSectiuni[1][1] != C.NU_AM_GASIT and centreSectiuni[1][0] != C.NU_AM_GASIT:
            self.vectorLatimiBanda[1] = centreSectiuni[1][1] - centreSectiuni[1][0]

        return self.vectorLatimiBanda

    def completareCentre(self, centreSectiuni, vectorLatimiMedii):  # completeaza centrele nedetectate din matricea centrelor cu ajutorul vectorLatimiBanda
        self.centreSectiuniCompletat = np.copy(centreSectiuni)

        if vectorLatimiMedii[0] != C.NU_AM_GASIT:
            if centreSectiuni[0][0] == C.NU_AM_GASIT and centreSectiuni[0][1] != C.NU_AM_GASIT:  # completare sectiuni sus
                self.centreSectiuniCompletat[0][0] = centreSectiuni[0][1] - vectorLatimiMedii[0]
                self.centreSectiuniCompletat[0][1] = centreSectiuni[0][1]
            elif centreSectiuni[0][0] != C.NU_AM_GASIT and centreSectiuni[0][1] == C.NU_AM_GASIT:
                self.centreSectiuniCompletat[0][0] = centreSectiuni[0][0]
                self.centreSectiuniCompletat[0][1] = centreSectiuni[0][0] + vectorLatimiMedii[0]

        if vectorLatimiMedii[1] != C.NU_AM_GASIT:
            if centreSectiuni[1][0] == C.NU_AM_GASIT and centreSectiuni[1][1] != C.NU_AM_GASIT:  # sectiune jos
                self.centreSectiuniCompletat[1][0] = centreSectiuni[1][1] - vectorLatimiMedii[1]
                self.centreSectiuniCompletat[1][1] = centreSectiuni[1][1]
            elif centreSectiuni[1][0] != C.NU_AM_GASIT and centreSectiuni[1][1] == C.NU_AM_GASIT:
                self.centreSectiuniCompletat[1][0] = centreSectiuni[1][0]
                self.centreSectiuniCompletat[1][1] = centreSectiuni[1][0] + vectorLatimiMedii[1]

        return self.centreSectiuniCompletat


