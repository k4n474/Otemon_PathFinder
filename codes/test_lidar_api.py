"""HTTP APIを実機・ネットワークなしで検証する。"""
import unittest
from unittest.mock import Mock, patch
from lidar_api import create_app
import park_control


class ViewerTest(unittest.TestCase):
    def test_viewer_and_cross_origin_points(self):
        reader = Mock(return_value=([], [], {"running": True, "fps": 12}))
        client = create_app(reader).test_client()
        page = client.get('/')
        self.assertEqual(page.status_code, 200)
        page.close()
        response = client.get('/api/points', headers={"Origin": "http://localhost:5501"})
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.headers['Access-Control-Allow-Origin'], 'http://localhost:5501')
        self.assertEqual(response.json['points'], [])
        self.assertEqual(response.json['fps'], 12)
        self.assertIsNone(response.json['front_wall'])
        self.assertIsNone(response.json['side_walls']['left'])
        self.assertTrue(client.get('/api/status').json['running'])

    def test_hardware_error_is_reported(self):
        client = create_app(Mock(side_effect=RuntimeError('serial unavailable'))).test_client()
        response = client.get('/api/points')
        self.assertEqual(response.status_code, 503)
        self.assertEqual(response.json['error'], 'serial unavailable')

    def test_parking_and_viewer_share_reader(self):
        reader = Mock(running=True)
        reader.get_points.return_value = []
        reader.get_status.return_value = {"fps": 10}
        park_control.prepare()
        with patch.object(park_control, '_lidar', reader):
            park_control._read_walls()
            response = create_app(park_control._read_lidar_frame).test_client().get('/api/points')
        reader.start.assert_not_called()
        self.assertEqual(reader.get_points.call_count, 2)
        self.assertEqual(response.status_code, 200)


if __name__ == '__main__':
    unittest.main()
