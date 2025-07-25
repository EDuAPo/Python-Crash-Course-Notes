# 测试函数
import unittest

def get_formatted_name(first, last):
    return f"{first.title()} {last.title()}"

class NamesTestCase(unittest.TestCase):
    def test_first_last_name(self):
        formatted_name = get_formatted_name('janis', 'joplin')
        self.assertEqual(formatted_name, 'Janis Joplin')

if __name__ == '__main__':
    unittest.main()