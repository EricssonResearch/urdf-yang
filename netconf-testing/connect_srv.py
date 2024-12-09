from ncclient import manager

m = manager.connect(host='192.168.1.100', port=830, username='netconf',
                    password='netconf', device_params={'name': 'csr'})

print m.connected