source /home/labarea-franka/Desktop/ceruzzi/venv/bin/activate
#source /mnt/f1e018d6-98bf-4b00-91de-f7b3f30ea52a/Paper_EAAI_2021/vasca-venv/bin/activate
cd ../build/unibas_insert_bh
#./scan3d 192.168.1.1 /mnt/f1e018d6-98bf-4b00-91de-f7b3f30ea52a/PegInHoleExperiments/d0107_deeplab_dati_video/
deactivate
source /home/labarea-franka/libfranka/unibas_insert_bh/unibas_inser_bh_venv/bin/activate 
cd ../../unibas_insert_bh/
python ply_multiway_registration.py /mnt/f1e018d6-98bf-4b00-91de-f7b3f30ea52a/PegInHoleExperiments/d0107_deeplab_dati_video/
python global_registration.py /mnt/f1e018d6-98bf-4b00-91de-f7b3f30ea52a/PegInHoleExperiments/d0107_deeplab_dati_video/
#
#
#source unibas_inser_bh_venv/bin/activate 
#cd ../build/unibas_insert_bh
#./scan3d 192.168.1.1
#cd ../../unibas_insert_bh/
#python ply_multiway_registration.py
#python global_registration.py
#cd ../build/unibas_insert_bh
#./big_head_insertion 192.168.1.1 /home/labarea-franka/libfranka/unibas_insert_bh/resources/output/fori_w.txt /mnt/f1e018d6-98bf-4b00-91de-f7b3f30ea52a/PegInHoleExperiments/d0107_deeplab_dati_video/ 2
#./big_head_insertion 192.168.1.1 /home/labarea-franka/libfranka/unibas_insert_bh/resources/output/fori_w.txt /home/labarea-franka/temp_ 1

#/home/labarea-franka/temp_
