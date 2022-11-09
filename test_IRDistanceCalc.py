# @author Chase Scott

import unittest
from IRDistanceCalc import distance


class MyTestCase(unittest.TestCase):
    def test_dis1(self):
        dis = float(distance(5.5, 7.25).split(',')[0])
        self.assertEqual(dis, 1)

    def test_dis2(self):
        dis = float(distance(15.88, 14.54).split(',')[0])
        self.assertAlmostEqual(dis, 14.9499, places=4)

    def test_ang1(self):
        ang = float(distance(5.5, 7.25).split(',')[1])
        self.assertAlmostEqual(ang, 52.1024, places=4)

    def test_ang2(self):
        ang = float(distance(15.88, 14.54).split(',')[1])
        self.assertAlmostEqual(ang, 104.0265, places=4)


if __name__ == '__main__':
    unittest.main()
