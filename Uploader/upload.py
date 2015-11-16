#!/usr/bin/env python
import sys
sys.path.insert(0, '/mnt/sda1/TangoLoggerUploader/requests' );
sys.path.insert(0, '/usr/lib/python2.7/bridge/')
import os
import re
import datetime
from bridgeclient import BridgeClient as bridgeclient

print("Starting Upload")
sys.stdout.flush()
    
client = bridgeclient()
client.put("is_uploading", "Y" )
print("Set Client Upload to Y")

print("importing requests")
sys.stdout.flush()
import requests
# url = "http://vm3.funkware.com/~altitude/TangoLogger.upload.cgi"
url = "http://207.111.254.90/upload_anon"
# data_dir = "/Users/tango/src/Arduino/TangoLoggerData"
data_dir = "/mnt/sd/TangoLoggerData"
a = os.listdir(data_dir)
#print(a)
for csv_file in a:
    if re.match(r'(\d\d\d\d\d-LG).CSV', csv_file):
        print ("File: " + csv_file + "."),
        sys.stdout.flush()
            
        upl_file = csv_file.replace("CSV", "UPL")
        full_file = data_dir + '/' + csv_file
        full_upl_file = data_dir + '/' + upl_file
        if os.path.isfile(full_upl_file):
            print ("UPL file " + upl_file + " exists.  Skipping")
	    sys.stdout.flush()
        else:
            file_info = os.stat(full_file)
            date_str = datetime.datetime.fromtimestamp(file_info.st_mtime).strftime('%Y_%m_%d-%H_%M_%S')
            remote_file = date_str + "-log.csv"
            print ("uploading as " + date_str + ". "),
	    sys.stdout.flush()
            files = {
                'tango_file': ('tango_file', open(full_file, 'rb'), 'text/csv')
            }
            data = {
                "input_data_type": "TangoLogger",
                "title": date_str,
                "visibiilty": "private",
            }
            headers = {
                "Host": "evviz.funkware.com:80"
            }
            r = requests.post(url, data=data, files=files, headers=headers)
            if r.status_code == 200:
                open(full_upl_file, 'a').close()
                print ("OK, creating UPL file")
            else:
                print ("Got return code: " + str(r.status_code) + ". Not setting UPL file")
	    sys.stdout.flush()
client.put("is_uploading", "N" )
print("Set Client Upload to N")
print("Done!")
