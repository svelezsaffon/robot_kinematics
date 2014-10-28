__author__ = 'santiago'


from itertools import izip

import xml.etree.ElementTree as ET
import numpy
import time

from openravepy import *

"""This two functions will help me calculate cosine and sine """


def cosine(angle):
    return numpy.cos(numpy.radians(angle))

def sine_d(angle):
    return numpy.sin(angle)

def cosine_d(angle):
    return numpy.cos(angle)

def sine(angle):
    return numpy.sin(numpy.radians(angle))

def arctan2(a,b):
    return numpy.radians(numpy.arctan2(a,b))

def arctan2x(a,b):
    return numpy.arctan2(numpy.radians(a),numpy.radians(b))


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
            lmap = self.get_link(link)
            return lmap[name]
        else:
            return None

    def get_paramf(self, link, name):
        return float(self.get_param(link,name))

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
        self.env=Environment()
        #self.env.SetViewer('qtcoin') # attach viewer (optional)
        self.env.Load('pumaarm.dae') # load a simple scene
        self.robot = self.env.GetRobots()[0]


        self.ARM=-1
        self.ARM2=0
        self.ELBOW=-1
        self.WRIST=1
        self.FLIP=1


    def fordward_kinematics_checkings(self):
        print '-----PERFORMING FORDWARD KINEMATICS CHECKING-----'

        for link in range(0,len(self.robot.GetLinks())):

            for value in range(-360,360):
                angle=sine(value)

                self.robot.SetDOFValues([angle],[link])
                print self.robot.GetLinks()[5].GetTransform()
                print "DOF"
                print self.robot.GetDOFValues()

                time.sleep(0.05)



    def pos_in_matrix(self,matrix,col,row):

        if col is 'n' or col is 'N':
            col = 0

        if col is 's' or col is 'S':
            col = 1

        if col is 'a' or col is 'A':
            col=2

        if col is 'p' or col is 'P':
            col =3

        if row is 'x' or row is 'X':
            row = 0

        if row is 'y' or row is 'Y':
            row = 1

        if row is 'z' or row is 'Z':
            row=2



        return float(matrix[int(row)][int(col)])


    def inverse_kinematics_tangent_half(self,end_pos):
        print "----PERFORMING INVERSE KINEMATICS - TANGENT HAL ANGLE-----"
        angles=[0,0,0,0,0,0]

        #we first calculate thetha 1, ans store it in an array
        angles[0]=self.theta_1_half_angle(end_pos)

        if angles[0] is not None:

            angles[2]=self.theta_3_half_angle(end_pos,angles)

            if angles[2] is not None:

                angles[1]=self.theta_2_half_angle(end_pos,angles)


        else:
            print "[TANGENT HALF ANGLE]Robot could not reach  THETA_1"



        return angles


    def theta_2_half_angle(self,end_pos,angles):
        return 10


    def theta_3_half_angle(self,end_pos,angles):
        a3=self.dh_table.get_paramf(3,'a')
        d4=self.dh_table.get_paramf(4,'d')
        a2=self.dh_table.get_paramf(2,'a')
        px=self.pos_in_matrix(end_pos,'p','x')
        py=self.pos_in_matrix(end_pos,'p','y')


        g214=cosine(angles[0])*px+sine(angles[0])*py
        g222=-self.pos_in_matrix(end_pos,'p','z')

        d=numpy.power(g214,2)+numpy.power(g222,2)-numpy.power(d4,2)-numpy.power(a3,2)-numpy.power(a2,2)
        e=(4*numpy.power(a2,2)*numpy.power(a3,2))+(4*numpy.power(a2,2)*numpy.power(d4,2))

        sqrt=e-numpy.power(d,2)

        if sqrt<0:
            sqrt*=-1

        if sqrt >=0 :
            return numpy.arctan2(2*a2*d4-self.ARM*self.ELBOW*numpy.sqrt(sqrt),d+2*a2*a3)


    def theta_1_half_angle(self,end_pos):
        px=self.pos_in_matrix(end_pos,'p','x')
        py=self.pos_in_matrix(end_pos,'p','y')
        d2=self.dh_table.get_paramf(2,'d')

        sqrt=numpy.power(px,2)+numpy.power(py,2)-numpy.power(d2,2)

        if sqrt >= 0:
            return 2*numpy.arctan2(-px-self.ARM*numpy.sqrt(sqrt),d2+py)
        else:
            return None

    def inverse_kinematics(self,end_pos):
        print "------PERFORMING INVERSE KINEMATICS---------"

        #This array contains all the 6 angles for the puma robot
        angles=[0,0,0,0,0,0]

        angles[0]=self.calculate_theta_1(end_pos)


        if angles[0] is not None:
            angles[2]=self.calculate_thetha_3(end_pos,angles)

            if angles[2] is not None:
                angles[1]=self.calculate_thetha_2(end_pos,angles)

                if angles[1] is not None:

                    angles[3]=self.calculate_thetha_4(end_pos,angles)

                    angles[4]=self.calculate_thetha_5(end_pos,angles)

                    angles[5]=self.calculate_theta_6(end_pos,angles)

                    self.move_robot_given_angles(angles)

                else:
                    print "Robot could not reach THETA_2 angle"
            else:
                print "Robot could not reach THETA_3 angle"
        else:
            print "Robot could not reach THETA_1 angle"

        return angles

    def calculate_theta_6(self,end_pos,angles):
        sx=self.pos_in_matrix(end_pos,'s','x')
        sy=self.pos_in_matrix(end_pos,'s','y')
        sz=self.pos_in_matrix(end_pos,'s','z')

        nx=self.pos_in_matrix(end_pos,'n','x')
        ny=self.pos_in_matrix(end_pos,'n','y')
        nz=self.pos_in_matrix(end_pos,'n','z')

        s1=sine(angles[0])
        c4=cosine(angles[3])
        c1=cosine(angles[0])
        c23=cosine(angles[1]+angles[2])
        s4=sine(angles[3])
        s23=sine(angles[1]+angles[2])

        return numpy.arctan2((-s1*c4-c1*c23*s4)*nx +(c1*c4-s1*c23*s4)*ny+s23*s4*nz ,(-s1*c4-c1*c23*s4)*sx + (c1*c4-s1*c23*s4)*sy + s23*s4*sz)


    def calculate_thetha_5(self,end_pos,angles):
        ax=self.pos_in_matrix(end_pos,'a','x')
        ay=self.pos_in_matrix(end_pos,'a','y')
        az=self.pos_in_matrix(end_pos,'a','z')

        c1=cosine(angles[0])
        s1=sine(angles[0])

        c4=cosine(angles[3])
        s4=sine(angles[3])

        c23=cosine(angles[1]+angles[2])
        s23=sine(angles[1]+angles[2])


        return numpy.arctan2((c1*c23*c4-s1*s4)*ax +(s1*c23*c4 + c1*s4)*ay-c4*s23*az,c1*s23*ax + s1*s23*ay + c23*az)




    def calculate_thetha_4(self,end_pos,angles):

        ax=self.pos_in_matrix(end_pos,'a','x')
        ay=self.pos_in_matrix(end_pos,'a','y')
        az=self.pos_in_matrix(end_pos,'a','z')

        a=self.WRIST*(cosine(angles[0])*ay - sine(angles[0])*ax)
        c23=cosine(angles[1]+angles[2])
        b=self.WRIST*(cosine(angles[0])*c23*ax  + sine(angles[0])*c23*ay -sine(angles[1]+angles[2])*az)


        return numpy.arctan2(a,b)


    def calculate_theta_1(self,end_pos):
        #In here we wil find the first angle Aplha1
        #All the calculations are base ont he equations from he book
        #-SIN(alpha_1)*Px+COS(alpha_1)*Py=D2
        #solving the equation using circle equation substitution we have
        px=self.pos_in_matrix(end_pos,'p','x')
        py=self.pos_in_matrix(end_pos,'p','y')

        d2=self.dh_table.get_paramf(2,'d')
        r=numpy.sqrt(numpy.power(px,2)+numpy.power(py,2))

        #theta1=atan2(py,px)-atan2(d2,+-sqrt(r^2-d_2^2)
        sqrt=numpy.power(r,2)-numpy.power(d2,2)

        if sqrt >= 0:
            return numpy.arctan2(py,px)-numpy.arctan2(d2,-self.ARM*numpy.sqrt(sqrt))
        else:
            return None



    def sign(self,value):
        if value >= 0:
            return 1
        else:
            return -1

    def calculate_thetha_2(self,end_pos,angles):
        px=self.pos_in_matrix(end_pos,'p','x')
        py=self.pos_in_matrix(end_pos,'p','y')
        d4=self.dh_table.get_paramf(4,'d')
        a2=self.dh_table.get_paramf(2,'a')
        a3=self.dh_table.get_paramf(3,'a')

        #f=cosine(angles[0])*px+sine(angles[0])*py
        f=(d4*cosine(angles[2])-a3*sine(angles[2]))*sine(angles[1])+(d4*sine(angles[3])+a3*cosine(angles[2])+a2)
        h=numpy.power(d4,2)+numpy.power(a2,2)+numpy.power(a3,2)+2*a2*d4*sine(angles[2])+2*a2*a3*cosine(angles[2])

        self.ARM2=self.sign( cosine(angles[1])*(d4*cosine(angles[2])-a3*sine(angles[2]))-sine(angles[1])*(d4*sine(angles[2])+a3*cosine(angles[2])+a2))

        sqrt=h-numpy.power(f,2)



        theta=None

        if sqrt >=0 :
            theta=numpy.arctan2(f,self.ARM2*numpy.sqrt(sqrt))-numpy.arctan2(d4*sine(angles[2])+a3*cosine(angles[2])+a2,d4*cosine(angles[2])-a3*sine(angles[2]))


        return theta




    def calculate_thetha_3(self,end_pos,angles):

        a3=self.dh_table.get_paramf(3,'a')
        d4=self.dh_table.get_paramf(4,'d')
        a2=self.dh_table.get_paramf(2,'a')
        px=self.pos_in_matrix(end_pos,'p','x')
        py=self.pos_in_matrix(end_pos,'p','y')



        g214=cosine_d(angles[0])*px+sine_d(angles[0])*py
        g222=-self.pos_in_matrix(end_pos,'p','z')


        #d=2*a2*d4*sine_d(angles[0])+2*a2*a3*cosine_d(angles[0])
        d=numpy.power(g214,2)+numpy.power(g222,2)-numpy.power(d4,2)-numpy.power(a3,2)-numpy.power(a2,2)

        e=(4*numpy.power(a2,2)*numpy.power(a3,2))+(4*numpy.power(a2,2)*numpy.power(d4,2))

        sqrt=numpy.abs(e-numpy.power(d,2))




        thetha=None

        if sqrt >= 0:

            thetha=numpy.arctan2(d,self.ARM*self.ELBOW*numpy.sqrt(sqrt))-numpy.arctan2(a3,d4)

        return thetha




    def move_robot_given_angles(self,angles):
        if len(angles)==self.dh_table.size():
            link=[0,1,2,3,4,5]
            self.robot.SetDOFValues(angles,link)




def print_menu():

    out = False

    deci=None
    while out is False:

        print "--------MENU FOR ROBOTICS PROGRAM----------"
        print "|Enter the number for the action you want |"
        print "|1)Display all trans. matrices            |"
        print "|2)Forward kinematic movement             |"
        print "|3)Inverse Kinematics-Circle Equation     |"
        print "|4)Inverse Kinematics-Tangent Half Angle  |"
        print "|6)Exit                                   |"
        print "|___________SANTIAGO_VELEZ_SAFFON_________|"

        deci=int(raw_input("Option="))

        if deci in range(1,7):
            out=True

    return deci



def main():

    deci=print_menu()


    if deci is not 6:
        robot = Puma560('dh_table.xml')

    while deci is not 6:

        if deci is 2:
            robot.fordward_kinematics_checkings()

        if deci is 3:

            pos=[
                [0.558560003,-0.829464118,0,0.105344877],
                [0.829464118,0.558560003,0,0.425164341],
                [0,0.0,1,1.80410000],
                [0,0,0,1]
            ]

            """
DOF
[  9.78147601e-01  -7.10542736e-15   5.32907052e-15   0.00000000e+00 0.00000000e+00   0.00000000e+00]
            """
            print robot.inverse_kinematics(pos)

            time.sleep(1)
            pos=[
                [0.93647051,-0.35074633,0,0.33271059],
                [0.35074633,0.93647051,0,0.28489634],
                [0,0.0,1,1.8041],
                [0,0,0,1]
            ]
            robot.inverse_kinematics(pos)

            time.sleep(1)
            pos=[
                [0.617488860,0.03488097,0.785805573,0.398891517],
                [-0.0210763584,0.999391074,-0.0278076892,0.143160538],
                [-0.786297205,0,0.617848148,1.81066334],
                [0,0,0,1]
            ]
            robot.inverse_kinematics(pos)

            time.sleep(1)
            pos=[
                [-0.26518816,0.03831466,0.96343511,0.77027201],
                [-0.01705003,0.99886756,-0.04441718,0.1366784],
                [-0.96404592,-0.02820551,-0.26423458,1.65252102],
                [0,0,0,1]
            ]
            robot.inverse_kinematics(pos)

        deci=print_menu()






if __name__ == '__main__':
    main()
