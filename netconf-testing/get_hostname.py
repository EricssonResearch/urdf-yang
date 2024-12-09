# Lekérdezés az aktuális konfigurációra
with manager.connect(host=HOST, port=PORT, username=USER, password=PASSWORD,
                     hostkey_verify=False) as m:
    
    # XML lekérdezés az aktuális hostname-re
    get_hostname = """
    <rpc xmlns="urn:ietf:params:xml:ns:netconf:base:1.0">
        <get>
            <filter type="subtree">
                <system xmlns="urn:ietf:params:xml:ns:yang:ietf-system"/>
            </filter>
        </get>
    </rpc>
    """
    
    response = m.get(filter=('subtree', get_hostname))
    print(response)