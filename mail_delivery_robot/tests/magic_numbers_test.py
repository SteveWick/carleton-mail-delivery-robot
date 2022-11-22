import unittest

from mail_delivery_robot.magicNumbers import magicNumbers, loadNumberOverrides

class MagicNumbersTest(unittest.TestCase):
    def test_load_numbers_overrides(self):
        test_magic_numbers = loadNumberOverrides()
        true_magic_numbers = magicNumbers
        self.assertEqual(test_magic_numbers, true_magic_numbers)

if __name__ == '__main__':
    unittest.main()
