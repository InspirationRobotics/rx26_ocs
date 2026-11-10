"""A live IP socket is visible without inventing a navigation heartbeat."""
import socket
from tests.harness import BridgeCase,wait_for
class TestIpConnection(BridgeCase):
    def test_identified_tether_socket_is_up_without_gps_and_drops_on_close(self):
        c=self.client();c.hello('UUV1','wifi',agent='graey_competition',synthesized=[])
        def vehicle():return next(v for v in self.bridge.snapshot()['vehicles'] if v['id']=='UUV1')
        self.assertTrue(wait_for(lambda:bool(vehicle()['links']['wifi'])))
        v=vehicle();self.assertTrue(v['links']['wifi']['connected']);self.assertEqual(v['report'],{})
        peer=self.bridge.tcp._peers['UUV1']['wifi']
        self.assertEqual(peer.sock.getsockopt(socket.SOL_SOCKET,socket.SO_KEEPALIVE),1)
        c.sock.shutdown(socket.SHUT_RDWR);c.close()
        self.assertTrue(wait_for(lambda:vehicle()['links']['wifi'] is None))
