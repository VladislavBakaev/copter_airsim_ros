import threading
import time
from decimal import Decimal

class ImuData():
    def __init__(self):

        self.linearNorthVel = Decimal(0)
        self.linearEastVel = Decimal(0)
        self.linearHeightVel = Decimal(0)

        self.linearNorthAcc = Decimal(0)
        self.linearEastAcc = Decimal(0)
        self.linearHeightAcc = Decimal(0)

        self.northIns = Decimal(0)
        self.heightIns = Decimal(0)
        self.eastIns = Decimal(0)

        thread = threading.Thread(target=self.integrateData, args=(Decimal(1/20),), daemon=True)
        thread.start()

    def update_date(self, linear_acc):
        self.linearNorthAcc = Decimal(linear_acc[0])
        self.linearEastAcc = Decimal(linear_acc[1])
        self.linearHeightAcc = Decimal(linear_acc[2])

    def integrateData(self, delay):
        while True:
            northVel = self.linearNorthVel + self.linearNorthAcc*delay
            eastVel = self.linearEastVel + self.linearEastAcc*delay
            heightVel = self.linearHeightVel + self.linearHeightAcc*delay

            self.northIns += Decimal(self.linearNorthVel*delay) + Decimal(self.linearNorthAcc*delay**2/2)
            self.eastIns += self.linearEastVel*delay + self.linearEastAcc*delay**2/2
            self.heightIns += self.linearHeightVel*delay + self.linearHeightAcc*delay**2/2

            self.linearNorthVel = northVel
            self.linearEastVel = eastVel
            self.linearHeightVel = heightVel

            time.sleep(float(delay))