# Convert pcap to ply and merge ply files from different sensors
1. descargar pcap y json config de ouster con ouster_connect.py (a dir con nombre de datetime de recording)
2. mkdir para cada sensor y mv json config por sensor a dir correspondiente (con pcap_to_ply.py)
3. pasar pcap a ply con pcap_to_ply.py para cada sensor (dar scan_num, va de 0 a scan_num)
4. seleccionar cantidad de archivos random y mergearlos con merge_ply_files.py 
