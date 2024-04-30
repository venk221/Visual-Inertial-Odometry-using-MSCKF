with open('stamped_groundtruth.txt','r') as f:
    gt=f.readlines()

with open('stamped_groundtruth.txt','r') as f:
    est=f.readlines()

print(len(gt))
print(len(est))

import numpy as np

error=0

for i in range(1,len(gt)):

    temp_err=0
    li_1=gt[i].split(' ')
    li_2=est[i].split(' ')

    print(li_1)



    x_1=float(li_1[1])
    x_2=float(li_2[1])

    temp_err+=np.sqrt((x_1-x_2)**2)

    y_1=float(li_1[2])
    y_2=float(li_2[2])

    temp_err+=np.sqrt((y_1-y_2)**2)

    z_1=float(li_1[3])
    z_2=float(li_2[3])

    temp_err+=np.sqrt((z_1-z_2)**2)

    qw_1=float(li_1[4])
    qw_2=float(li_2[4])

    temp_err+=np.sqrt((qw_1-qw_2)**2)

    qx_1=float(li_1[5])
    qx_2=float(li_2[5])

    temp_err+=np.sqrt((qx_1-qx_2)**2)

    qy_1=float(li_1[6])
    qy_2=float(li_2[6])

    temp_err+=np.sqrt((qy_1-qy_2)**2)


    qz_1=float(li_1[7])
    qz_2=float(li_2[7])

    temp_err+=np.sqrt((qz_1-qz_2)**2)


    #error+=

    break
