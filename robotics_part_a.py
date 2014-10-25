__author__ = 'santiago'


from itertools import izip

import xml.etree.ElementTree as ET
import math
import time
from openravepy import *

"""This two functions will help me calculate cosine and sine """


def cosine(angle):
    return (math.cos(math.radians(angle)))


def sine(angle):
    return (math.sin(math.radians(angle)))


"""
The purpose if this class is to read the file that contains the DH table,
I am doing an xml parser to give the user a flexible input. The xml file must contain the tags
    <link>
        <id>  </id>
        <thetha></thetha>
        <alpha></alpha>
        <a></a>
        <d></d>
    </link>
 """


class dh_table_loader():
    def __init__(self, file_name):
        self.file = file_name

    # This method is the one in charge of reading the file, parsing it
    # and creating a map that contains the DH table
    def read_file(self):
        dh_table_file = ET.parse(self.file)
        dh_root = dh_table_file.getroot()

        dh_table = {}
        for links in dh_root.findall('link'):
            link_map = {}

            link_i = int(links.find('id').text)  #link id
            link_map['thetha'] = links.find('thetha').text  #link theta angle
            link_map['alpha'] = links.find('alpha').text  #link aplha angle
            link_map['a'] = links.find('a').text  #link a distance
            link_map['d'] = links.find('d').text  #link d distance
            dh_table[link_i] = link_map

        return dh_table


"""
This class will help me manage everything on the DH table
"""


class DH_Table():
    def __init__(self):
        self.raw_table = {}

    def size(self):
        return len(self.raw_table)

    def load(self, file_name):
        loader = dh_table_loader(file_name)
        self.raw_table = loader.read_file()

    def get_link(self, index):
        if len(self.raw_table) >= index:
            return self.raw_table[index]
        else:
            return None

    """This method helps me to get the value of the given parameter on the given link"""

    def get_param(self, link, name):
        if len(self.raw_table) >= link:
            lmap = {}
            lmap = self.get_link(link)
            return lmap[name]
        else:
            return None

    """ This method helps me identify if the parameter is variable in the robot """

    def is_param_var(self, link, name):
        param = self.get_param(link, name)

        if param is not None:

            if param.find('=') >= 0:
                return True
            else:
                return False

        else:
            return None

    """
    This method calculates the i-1A 1  Matrxi
    """

    def A_i_matrix(self, link, theta=0, alpha=0, a=0, d=0):
        matrix = []
        thethavar = self.is_param_var(link, 'thetha')
        alphavar = self.is_param_var(link, 'alpha')
        avar = self.is_param_var(link, 'a')
        dvar = self.is_param_var(link, 'd')

        if alphavar is not None and thethavar is not None and avar is not None and dvar is not None:
            row = []
            row2 = []
            row3 = [0]
            row4 = [0, 0, 0, 1]

            if thethavar == False:
                theta = float(self.get_param(link, 'theta'))

            if alphavar == False:
                alpha = float(self.get_param(link, 'alpha'))

            if avar == False:
                a = float(self.get_param(link, 'a'))

            if dvar == False:
                d = float(self.get_param(link, 'd'))

            # ---Calculate Row 1

            row.append(cosine(theta))
            row.append(-1 * cosine(alpha) * sine(theta))
            row.append(sine(alpha) * sine(theta))
            row.append(a * cosine(theta))

            #---Calculate Row 2

            row2.append(sine(theta))
            row2.append(cosine(alpha) * cosine(theta))
            row2.append(-1 * sine(alpha) * cosine(theta))
            row2.append(a * sine(theta))

            #---Calculate Row 3
            row3.append(sine(alpha))
            row3.append(cosine(alpha))
            row3.append(d)

            #---Calculate Row 4

            matrix.append(row)
            matrix.append(row2)
            matrix.append(row3)
            matrix.append(row4)
        else:
            # Some of the link parameters where not found.... :(
            return None

        return matrix


    """This method is just to represent each matrix, instead of calculating the values it will show
        COS(ALPHA). Thsi is jus for debbugin
     """

    def give_Ai_matrix_rep(self, link):
        matrix = []
        thethavar = self.is_param_var(link, 'thetha')
        alphavar = self.is_param_var(link, 'alpha')
        avar = self.is_param_var(link, 'a')
        dvar = self.is_param_var(link, 'd')

        if alphavar is not None and thethavar is not None and avar is not None and dvar is not None:
            row = []
            row2 = []
            row3 = [0]
            row4 = [0, 0, 0, 1]

            if thethavar:
                row.append("COS(THETA_" + `link` + ")")
                row2.append("SIN(THETA_" + `link` + ")")

                if alphavar:
                    row.append("-COS(ALPHA_" + `link` + ")*SIN(THETA_" + `link` + ")")
                    row.append("SIN(ALPHA_" + `link` + ")*SIN(THETA_" + `link` + ")")

                    row2.append("COS(ALPHA_" + `link` + ")*COS(THETA_" + `link` + ")")
                    row2.append("-SIN(ALPHA_" + `link` + ")*COS(THETA_" + `link` + ")")

                    row3.append("SIN(ALPHA_" + `link` + ")")
                    row3.append("COS(ALPHA_" + `link` + ")")

                else:
                    alpha = float(self.get_param(link, 'alpha'))
                    sinalpha = sine(alpha)
                    cosalpha = cosine(alpha)
                    # #Colunm 2

                    if cosalpha != 0.0:
                        row2.append(`cosalpha` + "*COS(ALPHA_" + `link` + ")")
                        cosalpha = cosalpha * -1;
                        row.append(`cosalpha` + "*SIN(ALPHA_" + `link` + ")")
                    else:
                        row.append("0")
                        row2.append("0")

                    if sinalpha != 0.0:
                        row.append(`sinalpha` + "*SIN(ALPHA_" + `link` + ")")

                        sinalpha = sinalpha * -1;

                        row2.append(`sinalpha` + "*COS(ALPHA_" + `link` + ")")

                    else:
                        row.append("0")
                        row2.append("0")

                    row3.append(sinalpha)
                    row3.append(cosalpha)

                if avar:
                    row.append("a_" + `link` + "*COS(THETA_" + `link` + ")")
                    row2.append("a_" + `link` + "*SIN(THETA_" + `link` + ")")
                else:
                    a = float(self.get_param(link, 'a'))
                    if a != 0.0:
                        row.append(`a` + "*COS(THETA_" + `link` + ")")
                        row2.append(`a` + "*SIN(THETA_" + `link` + ")")
                    else:
                        row.append("0")
                        row2.append("0")



            else:
                # Angle theta does not varies in this link, must be a translational joint
                theta = float(self.get_param(link, 'thetha'))
                row.append(cosine(theta))
                row2.append(sine(theta))

                if alphavar:
                    #Angle alpha varies
                    sinealpha = sine(theta)

                    if sinealpha != 0.0:

                        if sinealpha < 0.0:
                            row.append("COS(ALPHA_" + `link` + ")*" + `sinealpha`)
                        else:
                            sinealpha = sinealpha * -1
                            row.append("-COS(ALPHA_" + `link` + ")*" + `sinealpha`)
                    else:
                        row.append("0")

                    cosatheta = cosine(theta)

                    if cosatheta != 0.0:
                        row2.append("COS(ALPHA_" + `link` + ")*" + `cosatheta`)
                    else:
                        row2.append('0')


                else:
                    alpha = float(self.get_param(link, 'alpha'))
                    #colunm 2
                    row.append(-1 * cosine(alpha) * sine(theta))
                    row2.append(cosine(alpha) * cosine(theta))
                    row3.append(sine(alpha))

                    #column 3

                    row.append(sine(alpha) * sine(theta))
                    row2.append(-1 * sine(alpha) * cosine(theta))
                    row3.append(cosine(alpha))



                    #column 4

                if avar:
                    row.append("a_" + `link` + '*' + `sine(theta)`)
                    row2.append("a_" + `link` + '*' + `cosine(theta)`)
                else:
                    a = float(self.get_param(link, 'a'))
                    row.append(a * sine(theta))
                    row2.append(a * cosine(theta))

            if dvar:
                row3.append("d_" + `link`)
            else:
                row3.append(self.get_param(link, 'd'))

            matrix.append(row)
            matrix.append(row2)
            matrix.append(row3)
            matrix.append(row4)

            return matrix



        else:
            return None


"""This Class is created only to do math with matrices, all fo its methods are static so we dont need
    to instantiate an object
"""


class math_matrix(object):
    @staticmethod
    def multiply(A, B):
        matrix = []

        for row in range(0, 4):
            matrix.append([])
            for col in range(0, 4):
                matrix[row].append(A[row][col] * B[col][row])

        return matrix


class Puma560(object):
    def __init__(self, param_file):
        self.dh_table = DH_Table()
        self.dh_table.load(param_file)



    def fordward_kinematics_checkings(self):
        print '-----PERFORMING FORDWARD KINEMATICS CHECKING-----'
        env=Environment()
        env.SetViewer('qtcoin') # attach viewer (optional)
        env.Load('pumarobot.xml') # load a simple scene

        robot = env.GetRobots()[0]

        var =1
        while True :

            var=var+0.1



            i=1
            first =True
            for link in robot.GetLinks():
                if i==1:
                    mat=self.dh_table.A_i_matrix(i,var,None,0,1)
                    print link.GetTransform()
                    #link.SetTransform(mat)

                i=i+1


            time.sleep(25)




def main():
    robot = Puma560('dh_table.xml')
    robot.fordward_kinematics_checkings()


if __name__ == '__main__':
    main()
