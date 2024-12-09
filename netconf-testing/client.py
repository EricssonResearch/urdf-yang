from ncclient import manager

# NETCONF szerver elérhetősége
HOST = '192.168.1.100'  # A NETCONF szerver IP címe
PORT = 830
USER = 'netconf'  # NETCONF felhasználónév
PASSWORD = 'netconf'  # NETCONF jelszó

# Csatlakozás a NETCONF szerverhez
with manager.connect(host=HOST, port=PORT, username=USER, password=PASSWORD,
                     hostkey_verify=False) as m:

    # XML payload a hostname módosításához
    hostname_change = """
    <rpc xmlns="urn:ietf:params:xml:ns:netconf:base:1.0">
        <edit-config>
            <target>
                <running/>
            </target>
            <config>
                <system xmlns="urn:ietf:params:xml:ns:yang:ietf-system">
                    <hostname>new-hostname</hostname>
                </system>
            </config>
        </edit-config>
    </rpc>
    """

    # Módosítás végrehajtása
    response = m.edit_config(target='running', config=hostname_change)

    print(response)