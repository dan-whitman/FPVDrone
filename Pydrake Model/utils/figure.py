##### OTHER IMPORTS
import matplotlib.pyplot as plt

##### PLOTTING FUNCTION (plots figures to keep main code clean, returns figures for saving)
def PlotData(plotdata: list) -> None:
    # extracting plotdata (bad way to do this, rethink later)
    [times, camera_headings, us, vs, camera_width, camera_height] = plotdata

    # camera heading figure
    figheadings, axheadings = plt.subplots()
    axheadings.plot(times, camera_headings[:,0], label=r'$x$')
    axheadings.plot(times, camera_headings[:,1], label=r'$y$')
    axheadings.plot(times, camera_headings[:,2], label=r'$z$')
    axheadings.set_xlabel(r'Time ($s$)')
    axheadings.set_ylabel('Camera heading component')
    figheadings.suptitle('Camera Heading in World Frame', fontsize=15)
    axheadings.set_title('(Simulation)')
    axheadings.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    axheadings.grid()
    # figheadings.show()

    # target pixel measurement figure
    targetoutidx = [idx for idx, u in enumerate(us) if u == None]
    targetoutbins = VBinIdx(targetoutidx)

    figtargetpixels, axtargetpixels = plt.subplots()
    axtargetpixels.plot(times, us, label=r'$u$', color='blue')
    axtargetpixels.plot(times, vs, label=r'$v$', color='green')
    axtargetpixels.hlines(camera_width, xmin=times[0], xmax=times[-1], label='Camera width', color='blue', linestyles='--')
    axtargetpixels.hlines(camera_height, xmin=times[0], xmax=times[-1], label='Camera height', color='green', linestyles='--')
    axtargetpixels.axhline(0, color='black')
    if targetoutbins: # if bins
        for bin in targetoutbins: # add dropout bins
            plt.axvspan(times[bin[0]], times[bin[1]], color='red', alpha=0.5)
    axtargetpixels.set_xlabel(r'Time ($s$)')
    axtargetpixels.set_ylabel(r'Target pixel measurements ($px$)')
    figtargetpixels.suptitle('Target Pixel Measurements in Camera Sensor', fontsize=15)
    axtargetpixels.set_title('(Simulation)')
    axtargetpixels.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    axtargetpixels.grid()
    # figtargetpixels.show()

    # return all figures to be saved
    return figheadings, figtargetpixels

##### VBLOCK INDEX RETURN FUNCTION (input idx range, outputs idx bins)
def VBinIdx(idx: list) -> list:
    idxblocks = []

    if idx: # if the list is not empty
        i = 0

        while i < len(idx): # while inside the list
            idxblocks.append(idx[i]) # add element i to bin start
            if i != len(idx) - 1: # if not at last element
                for j in range(1, len(idx) - i): # loop over remaining elements
                    if idx[i+j] != idx[i] + j: # if not consecutive
                        idxblocks.append(idx[i+j-1]) # add element i+j-1 to bin end
                        i = i + j # update i
                        break
                    if i + j == len(idx) - 1: # if at last element
                        idxblocks.append(idx[i+j]) # add last element to bin
                        i = len(idx) # update i to break while
                        break
            else: break # if at last element

    if len(idxblocks) % 2 == 1: idxblocks.append(-1) # if odd, append -1
    idxblocks = [idxblocks[i:i+2] for i in range(0,len(idxblocks),2)] # group bins

    return idxblocks

