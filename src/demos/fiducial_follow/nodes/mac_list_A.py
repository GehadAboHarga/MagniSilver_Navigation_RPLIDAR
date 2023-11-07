import requests
from threading import Thread
from time import sleep
from requests.packages.urllib3.exceptions import InsecureRequestWarning

requests.packages.urllib3.disable_warnings(InsecureRequestWarning)

MAC_list_status = {
  
}

MAC_list_role = {
    
}
MAC_list_name= {
    }

def init_state():
    global MAC_list_status
    global MAC_list_role
    global MAC_list_name
    MAC_list_status = {
    '70:B3:D5:A4:3B:EE': 'DOWN',
    '70:B3:D5:7D:30:1A': 'DOWN',
    '70:B3:D5:7D:31:7E': 'DOWN',
    '70:B3:D5:A4:3B:EC': 'DOWN',
    '70:B3:D5:A4:3B:74': 'DOWN'
    
    }

                    
    MAC_list_role = {
        '70:B3:D5:A4:3B:EE': 'Destination',
        '70:B3:D5:7D:30:1A': 'passby',
        '70:B3:D5:7D:31:7E': 'passby',
        '70:B3:D5:A4:3B:EC': 'passby',
        '70:B3:D5:A4:3B:74': 'passby'
        
    }
    MAC_list_name = {
        '70:B3:D5:A4:3B:EE': 'A',
        '70:B3:D5:7D:30:1A': 'B', 
        '70:B3:D5:7D:31:7E': 'C',
        '70:B3:D5:A4:3B:74': 'D',
        '70:B3:D5:A4:3B:EC': 'E'
    }


keep_looping =True
    
def get_mac_list(arg):
    global MAC_list_status
    global MAC_list_role
    global MAC_list_name
    html_content = ''
    ap_ip = '192.168.0.190'
    try:

        login_url = 'https://'+ap_ip+'/api/auth/signin'  # Replace with the login page URL
        list_secure_url = 'https://'+ap_ip+'/api/auth/eps'
        payload = {
            'userName': 'admin',
            'password': 'password'
        }
        session = requests.Session()
        html_text = session.post(login_url, data=payload, verify=False)
        html_content = html_text. json()
        #print(f"html_text={html_content['token']}")
        headers= {
                  'Authorization': "Basic " +  html_content['token']
               }

        # Step 3: Use the session cookies for subsequent requests
        #response = session.get(secure_url, verify=False) 
        response = session.patch(list_secure_url, headers=headers , verify=False)
        response = session.get(list_secure_url, headers=headers , verify=False)
        # Step 4: Retrieve the HTML content
        html_content = response.json()
        init_state()
        for item in html_content:
            #print(f'item = {item["mac"]}, {item["state"]}')
            MAC_list_status[item["mac"]] = str(item["state"]) +''
        
        #print(f'==============================')
        #print(f'html_content={html_content}')
        #print(f'==============================')
    except Exception as e:
        print(e)
        pass
    
    return MAC_list_status
def stop_keep_looping(flag):
    global keep_looping
    global thread
    keep_looping = not flag
    thread.join()
    return 
def loop_get_mac_list(arg):
    global MAC_list_status
    global MAC_list_role
    global MAC_list_name
    global keep_looping
    init_state()
    while (keep_looping == True):
        
        MAC_list_status = get_mac_list(arg)
        print('==============================')
        print(MAC_list_status )
        print('==============================')
        #print(MAC_list_role)
        for mac, state in MAC_list_status.items():
            #print(mac, state)
            if state == 'UP':
                #print( mac, MAC_list_role[mac])
                pass
        #sleep(1)
#loop_get_mac_list(10)

def get_mac_list_():
    global MAC_list_status
    return MAC_list_status
thread = Thread(target = loop_get_mac_list, args = (10, ))
thread.start()
    
if __name__ == "__main__":
    thread = Thread(target = loop_get_mac_list, args = (10, ))
    thread.start()
    #thread.join()
    
    print("thread finished...exiting")
