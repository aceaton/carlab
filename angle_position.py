from position_utils import *
# using two numpy arrays of same len
# gives t3 degrees clockwise from t2
def get_ang_2(r2,r3):

    # plt.close('all')
    a = np.clip(r2,np.amin(r2),(np.amax(r2)-np.amin(r2))*clp+np.amin(r2))
    b = np.clip(r3,np.amin(r3),(np.amax(r3)-np.amin(r3))*clp+np.amin(r3))
    b2 = np.flip(b[:int(arr_size/2)])
    r32 = np.flip(r3[:int(arr_size/2)])

    filt=np.convolve(a,b2,mode='valid')#full')[45:105]#[int(arr_size/2):int(arr_size)]  
    filt2=np.convolve(r2,r32,mode='valid')#[45:105]#[int(arr_size/2):int(arr_size)]  
    
    # plt.figure(2)
    # plt.plot(r2,label='r2')
    # plt.plot(r3,label='r3')
    # plt.plot(a,label='r2')
    # plt.plot(b,label='r3')

    # # plt.plot(np.flip(b[:int(arr_size/2)]))
    # plt.figure(0)
    # plt.plot(filt)
    # plt.figure(1)
    # plt.plot(filt2)
    # plt.show()
    # print("PLOTTED")

    ps1 = find_peaks(filt,distance=10,prominence=600)
    ps = ps1[0]
    print(ps)
    # ps = ps + len(b) - 1
    # print(ps)
    diff = np.mean(np.diff(ps))
    # ps[1]-ps[0]
    perc = 1-ps[0]/(diff)
    # print('t3 is ' + str(()*360) + ' degrees clockwise from t2')
    # print(ps1[1])
    return perc

def get_pos_ang_3(r2,r3,r4,p2,p3,p4):
    a1 = get_ang_2(r2,r3)
    a2 = get_ang_2(r3,r4)
    return calc_ang_pos(p2,p3,p4,a1,a2)

# def get_angle_arrs_3(window=100):
#     r0 = np.empty(window,dtype=int)
#     r1 = np.empty(window,dtype=int)
#     r2 = np.empty(window,dtype=int)
#     for (i=0;i<window;i+=1):
#         line = ser.readline()
#         if len(line) ==0:
#             print("angle receiver timeout")
#             sys.exit()
#         r0[i] = line
#         line = ser.readline()

# def get_ice_cream()


# given a car facing in the direction 0,1 on the point 0,0
# def plot_traj(targ=[0,0],l_rad=10,r_rad=10): 