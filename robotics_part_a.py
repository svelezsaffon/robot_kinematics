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
            aux=[]
            for col in range(0, 4):
                aux.append(A[row][col] * B[col][row])
            matrix.append(aux)
        return matrix


class Puma560(object):
    def __init__(self, param_file):
        self.dh_table = DH_Table()
        self.dh_table.load(param_file)
        self.env=Environment()
        self.env.SetViewer('qtcoin') # attach viewer (optional)
        self.env.Load('pumaarm.dae') # load a simple scene
        self.robot = self.env.GetRobots()[0]


        t1=numpy.radians(self.dh_table.get_paramf(1,'thetha'))
        t2=numpy.radians(self.dh_table.get_paramf(2,'thetha'))
        t3=numpy.radians(self.dh_table.get_paramf(3,'thetha'))
        t4=numpy.radians(self.dh_table.get_paramf(4,'thetha'))
        t5=numpy.radians(self.dh_table.get_paramf(5,'thetha'))
        t6=numpy.radians(self.dh_table.get_paramf(6,'thetha'))

        initpos=[t1,t2,t3,t4,t5,t6]

        self.ARM=-1
        self.ARM2=0
        self.ELBOW=-1
        self.WRIST=1
        self.FLIP=1

        self.update_indicators(initpos)




    def test(self):
        newdof=[numpy.radians(90),  #1
                numpy.radians(0),   #2
                numpy.radians(90),  #3
                numpy.radians(0),   #4
                numpy.radians(0),   #5
                numpy.radians(0)    #6
        ]

        self.robot.SetDOFValues(newdof,[0,1,2,3,4,5])

        mat=self.create_t6_matrix(newdof)

        for line in  mat:
            print line

        mat=self.robot.GetLinks()[6].GetTransform()

        for line in  mat:
            print line

    def create_t6_matrix(self,angles):
        c1=cosine_d(angles[0])
        c2=cosine_d(angles[1])
        c4=cosine_d(angles[3])
        c5=cosine_d(angles[4])
        c6=cosine_d(angles[5])

        c23=cosine_d(angles[1]+angles[2])

        s1=sine_d(angles[0])
        s2=sine_d(angles[1])
        s3=sine_d(angles[2])
        s4=sine_d(angles[3])
        s5=sine_d(angles[4])
        s6=sine_d(angles[5])
        s23=sine_d(angles[1]+angles[2])

        d6=self.dh_table.get_paramf(6,'d')
        d4=self.dh_table.get_paramf(4,'d')
        d2=self.dh_table.get_paramf(2,'d')
        a2=self.dh_table.get_paramf(2,'a')
        a3=self.dh_table.get_paramf(3,'a')


        matrix=[
            [1,0,0,0],
            [0,1,0,0],
            [0,0,1,0],
            [0,0,0,1],
        ]

        ##Normal vector  N
        val=c1*(c23*(c4*c5*c6-s4*s6)-s23*s5*c6)-s1*(s4*c5*c6+c4*s6)
        #if numpy.abs(val)< 0.000000001:
        #    val=0.0

        matrix[0][0]=val

        val=s1*(c23*(c4*c5*c6-s4*s6)-s23*s5*c6)-c1*(s4*c5*c6+c4*s6)
        #if numpy.abs(val)< 0.000000001:
        #    val=0.0
        matrix[1][0]=val

        val=-s23*(c4*c5*c6-s4*s5)-c23*s5*c6
        #if numpy.abs(val)< 0.000000001:
        #    val=0.0
        matrix[2][0]=val



        ##Sliding vector S
        val=c1*(-c23*(c4*c5*c6+s4*c6)+s23*s5*s6)-s1*(-s4*c5*s6+c4*c6)
        #if numpy.abs(val)< 0.000000001:
        #    val=0.0

        matrix[0][1]=val

        val=s1*(-c23*(c4*c5*c6+s4*c6)+s23*s5*s6)-c1*(-s4*c5*s6+c4*c6)
        #if numpy.abs(val)< 0.000000001:
        #    val=0.0

        matrix[1][1]=val

        val=s23*(c4*c5*s6+s4*c6)+c23*s5*s6
        #if numpy.abs(val)< 0.000000001:
        #    val=0.0
        matrix[2][1]=val


        #approach Vector A
        val=c1*(c23*c4*s5+s23*c5)-s1*s4*s5
        #if numpy.abs(val)< 0.000000001:
        #    val=0.0
        matrix[0][2]=val


        val=s1*(c23*c4*s5+s23*c5)-c1*s4*s5
        #if numpy.abs(val)< 0.000000001:
        #    val=0.0
        matrix[1][2]=val


        val=-s23*c4*s5+c23*c5
        #if numpy.abs(val)< 0.000000001:
        #    val=0.0
        matrix[2][2]=val

        #Psoition vector P

        val=c1*(d6*(c23*c4*s5+s23*c5)+s23*d4+a3*c23+a2*c2)-s1*(d6*s4*s5+d2)
        #if numpy.abs(val)< 0.000000001:
        #    val=0.0

        matrix[0][3]=val


        val=s1*(d6*(c23*c4*s5+s23*c5)+s23*d4+a3*c23+a2*c2)-c1*(d6*s4*s5+d2)
        #if numpy.abs(val)< 0.000000001:
        #    val=0.0
        matrix[1][3]=val

        val=d6*(c23*c5-s23*c4*s5)+c23*d4-a3*s23-a2*s2
        #if numpy.abs(val)< 0.000000001:
        #    val=0.0

        matrix[2][3]=val


        return  matrix


    def fordward_kinematics_checkings(self,write_to_file=False):
        print '-----PERFORMING FORDWARD KINEMATICS CHECKING-----'

        init=[0,0,0,0,0,0]

        self.robot.SetDOFValues(init,[0,1,2,3,4,5])

        long=40

        file=None
        if write_to_file:
            file=open("points.txt",'w')
            file.write(str((long*2)-2))
            file.write("\n")


        for value in range(-long,long):

            for link in range(0,len(self.robot.GetLinks())-1):


                angle=numpy.radians(value)
                self.robot.SetDOFValues([angle],[link])

                time.sleep(0.01)


            if write_to_file:
                t=self.robot.GetLinks()[6].GetTransform()
                dof=self.robot.GetActiveDOFValues()

                for col in range(0,4):
                    for row in range(0,4):
                            file.write(str(`t[col][row]`+" "))

                file.write("\n")

                for val in dof:
                    file.write(`dof[val]`+" ")

                file.write("\n")



        if write_to_file:
            file.close()

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



        return float(matrix[row][col])

    def inverse_kinematics_tangent_half_top(self):
        pos=[]

        file=open("points.txt",'r')

        lines=int(file.readline().split(" ")[0])

        eror=open("error_half.txt",'w')

        errortot=[0,0,0,0,0,0]

        for line in range(0,lines):


            mat=file.readline().split(" ")
            dofl=file.readline().split(" ")

            dof=[]
            for aux in range(0,len(dofl)-1):
                dof.append(float(dofl[aux]))

            pos=[]
            col=[0,0,0,0]
            aux=0
            for i in range(0,len(mat)-1):
                col[aux]=float(mat[i])
                aux+=1
                if aux == 4:
                    pos.append(col)
                    col=[0,0,0,0]
                    aux=0

            pos=pos
            errorlocal=[0,0,0,0,0,0]

            self.update_indicators(dof)

            sol=self.inverse_kinematics_tangent_half(pos,dof[2])

            if sol[0] is not None and sol[1] is not None and sol[2] is not None and sol[3] is not None and sol[4] is not None and sol[5] is not None:

                self.move_robot_given_angles(sol)

                for i in range(0,len(dof)) :
                    errorlocal[i]=dof[i]-sol[i]
                    eror.write(str(errorlocal))
                    eror.write("\n")
                    errortot[i]+=errorlocal[i]

                time.sleep(0.09)

            else:
                print "ooops"



        eror.write(str(errortot))

        eror.close()
        file.close()


    def inverse_kinematics_tangent_half(self,end_pos,thetha):
        print "----PERFORMING INVERSE KINEMATICS - TANGENT HAL ANGLE-----"
        angles=[0,0,0,0,0,0]

        #we first calculate thetha 1, ans store it in an array
        angles[0]=self.theta_1_half_angle(end_pos)

        if angles[0] is not None:

            angles[2]=thetha

            if angles[2] is not None:

                angles[1]=self.theta_2_half_angle(end_pos,angles)

                if angles[1] is not None:

                    angles[3]=self.theta_4_half_angle(end_pos,angles)

                    angles[4]=self.theta_5_half_angle(end_pos,angles)

                    angles[5]=self.theta_6_half_angle(end_pos,angles)


                else:
                    print "[TANGENT HALF ANGLE]Robot could not reach  THETA_2"

            else:
                print "[TANGENT HALF ANGLE]Robot could not reach  THETA_3"
        else:
            print "[TANGENT HALF ANGLE]Robot could not reach  THETA_1"



        return angles


    def theta_5_half_angle(self,end_pos,angles):
        ax=self.pos_in_matrix(end_pos,'a','x')
        ay=self.pos_in_matrix(end_pos,'a','y')
        az=self.pos_in_matrix(end_pos,'a','z')

        c1=cosine_d(angles[0])
        s1=sine_d(angles[0])

        c4=cosine_d(angles[3])
        s4=sine_d(angles[3])

        c23=cosine_d(angles[1]+angles[2])
        s23=sine_d(angles[1]+angles[2])


        return numpy.arctan2((c1*c23*c4-s1*s4)*ax +(s1*c23*c4 + c1*s4)*ay-c4*s23*az,c1*s23*ax + s1*s23*ay + c23*az)


    def theta_6_half_angle(self,end_pos,angles):
        sx=self.pos_in_matrix(end_pos,'s','x')
        sy=self.pos_in_matrix(end_pos,'s','y')
        sz=self.pos_in_matrix(end_pos,'s','z')

        nx=self.pos_in_matrix(end_pos,'n','x')
        ny=self.pos_in_matrix(end_pos,'n','y')
        nz=self.pos_in_matrix(end_pos,'n','z')

        s1=sine_d(angles[0])
        c4=cosine_d(angles[3])
        c1=cosine_d(angles[0])
        c23=cosine_d(angles[1]+angles[2])
        s4=sine_d(angles[3])
        s23=sine_d(angles[1]+angles[2])

        return numpy.arctan2((-s1*c4-c1*c23*s4)*nx +(c1*c4-s1*c23*s4)*ny+s23*s4*nz ,(-s1*c4-c1*c23*s4)*sx + (c1*c4-s1*c23*s4)*sy + s23*s4*sz)

    def update_indicators(self,angles):
        print angles

    def theta_4_half_angle(self,end_pos,angles):

        ax=self.pos_in_matrix(end_pos,'a','x')
        ay=self.pos_in_matrix(end_pos,'a','y')
        az=self.pos_in_matrix(end_pos,'a','z')

        a=self.WRIST*(cosine_d(angles[0])*ay - sine_d(angles[0])*ax)
        c23=cosine_d(angles[1]+angles[2])
        b=self.WRIST*(cosine_d(angles[0])*c23*ax  + sine_d(angles[0])*c23*ay -sine_d(angles[1]+angles[2])*az)

        return numpy.arctan2(a,b)

    def theta_2_half_angle(self,end_pos,angles):
        px=self.pos_in_matrix(end_pos,'p','x')
        py=self.pos_in_matrix(end_pos,'p','y')
        d4=self.dh_table.get_paramf(4,'d')
        a2=self.dh_table.get_paramf(2,'a')
        a3=self.dh_table.get_paramf(3,'a')


        c1=cosine_d(angles[0])
        s1=sine_d(angles[0])
        c3=cosine_d(angles[2])
        s3=sine_d(angles[2])

        theta2dh=self.dh_table.get_paramf(2,'thetha')
        theta3dh=self.dh_table.get_paramf(3,'thetha')

        f=c1*px+s1*py

        h=numpy.power(d4,2)+numpy.power(a2,2)+numpy.power(a3,2)+2*a2*d4*sine_d(angles[2])+2*a2*a3*cosine_d(angles[2])


        sqrt=(h-numpy.power(f,2))

        theta=None

        if sqrt >=0 :
            theta=2*numpy.arctan2((d4*c3-a3*s3)-self.ARM2*numpy.sqrt(sqrt),f+(d4*s3+a3*c3+a2))

        return theta




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

        sqrt=numpy.abs(e-numpy.power(d,2))

        thetha=None

        if sqrt >= 0:

            thetha=2*numpy.arctan2(2*a2*d4-self.ARM*self.ELBOW*numpy.sqrt(sqrt),d+2*a2*a3)

        return thetha


    def theta_1_half_angle(self,end_pos):
        px=self.pos_in_matrix(end_pos,'p','x')
        py=self.pos_in_matrix(end_pos,'p','y')
        d2=self.dh_table.get_paramf(2,'d')

        sqrt=numpy.power(px,2)+numpy.power(py,2)-numpy.power(d2,2)

        if sqrt >= 0:
            return 2*numpy.arctan2(-px-self.ARM*numpy.sqrt(sqrt),d2+py)
        else:
            return None

    def inverse_kinematics_top(self):
        pos=[]

        file=open("points.txt",'r')

        lines=int(file.readline().split(" ")[0])

        eror=open("error.txt",'w')

        errortot=[0,0,0,0,0,0]

        for line in range(0,lines):


            mat=file.readline().split(" ")
            dofl=file.readline().split(" ")

            dof=[]
            for aux in range(0,len(dofl)-1):
                dof.append(float(dofl[aux]))

            pos=[]
            col=[0,0,0,0]
            aux=0
            for i in range(0,len(mat)-1):
                col[aux]=float(mat[i])
                aux+=1
                if aux == 4:
                    pos.append(col)
                    col=[0,0,0,0]
                    aux=0

            pos=pos

            errorlocal=[0,0,0,0,0,0]


            self.update_indicators(dof)

            sol=self.inverse_kinematics(pos,dof[2])


            if sol[0] is not None and sol[1] is not None and sol[2] is not None and sol[3] is not None and sol[4] is not None and sol[5] is not None:

                self.move_robot_given_angles(sol)

                for i in range(0,len(dof)) :
                    errorlocal[i]=dof[i]-sol[i]
                    eror.write(str(errorlocal))
                    eror.write("\n")
                    errortot[i]+=errorlocal[i]

                time.sleep(0.09)

            else:
                print "oops"



        eror.write(str(errortot))

        eror.close()
        file.close()

    def inverse_kinematics(self,end_pos,theta):
        print "------PERFORMING INVERSE KINEMATICS---------"

        #This array contains all the 6 angles for the puma robot
        angles=[0,0,0,0,0,0]

        angles[0]=self.calculate_theta_1(end_pos)


        if angles[0] is not None:
            angles[2]=theta #self.calculate_thetha_3(end_pos,angles)

            if angles[2] is not None:

                angles[1]=self.calculate_thetha_2(end_pos,angles)

                if angles[1] is not None:

                    angles[3]=self.calculate_thetha_4(end_pos,angles)

                    angles[4]=self.calculate_thetha_5(end_pos,angles)

                    angles[5]=self.calculate_theta_6(end_pos,angles)

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

        s1=sine_d(angles[0])
        c4=cosine_d(angles[3])
        c1=cosine_d(angles[0])
        c23=cosine_d(angles[1]+angles[2])
        s4=sine_d(angles[3])
        s23=sine_d(angles[1]+angles[2])

        return numpy.arctan2((-s1*c4-c1*c23*s4)*nx +(c1*c4-s1*c23*s4)*ny+s23*s4*nz ,(-s1*c4-c1*c23*s4)*sx + (c1*c4-s1*c23*s4)*sy + s23*s4*sz)


    def calculate_thetha_5(self,end_pos,angles):
        ax=self.pos_in_matrix(end_pos,'a','x')
        ay=self.pos_in_matrix(end_pos,'a','y')
        az=self.pos_in_matrix(end_pos,'a','z')

        c1=cosine_d(angles[0])
        s1=sine_d(angles[0])

        c4=cosine_d(angles[3])
        s4=sine_d(angles[3])

        c23=cosine_d(angles[1]+angles[2])
        s23=sine_d(angles[1]+angles[2])


        return numpy.arctan2((c1*c23*c4-s1*s4)*ax +(s1*c23*c4 + c1*s4)*ay-c4*s23*az,c1*s23*ax + s1*s23*ay + c23*az)




    def calculate_thetha_4(self,end_pos,angles):

        ax=self.pos_in_matrix(end_pos,'a','x')
        ay=self.pos_in_matrix(end_pos,'a','y')
        az=self.pos_in_matrix(end_pos,'a','z')

        a=self.WRIST*(cosine_d(angles[0])*ay - sine_d(angles[0])*ax)
        c23=cosine_d(angles[1]+angles[2])
        b=self.WRIST*(cosine_d(angles[0])*c23*ax  + sine_d(angles[0])*c23*ay -sine_d(angles[1]+angles[2])*az)


        return numpy.arctan2(a,b)


    def calculate_theta_1(self,end_pos):
        #In here we wil find the first angle Aplha1
        #All the calculations are base ont he equations from he book
        #-SIN(alpha_1)*Px+COS(alpha_1)*Py=D2
        #solving the equation using circle equation substitution we have
        px=self.pos_in_matrix(end_pos,'p','x')
        py=self.pos_in_matrix(end_pos,'p','y')

        d2=self.dh_table.get_paramf(2,'d')
        r=numpy.power(px,2)+numpy.power(py,2)

        sqrt=r-numpy.power(d2,2)

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


        c1=cosine_d(angles[0])
        s1=sine_d(angles[0])

        theta2dh=self.dh_table.get_paramf(2,'thetha')
        theta3dh=self.dh_table.get_paramf(3,'thetha')

        f=c1*px+s1*py

        h=numpy.power(d4,2)+numpy.power(a2,2)+numpy.power(a3,2)+2*a2*d4*sine_d(angles[2])+2*a2*a3*cosine_d(angles[2])

        #self.ARM2=self.sign(cosine(theta2dh)*(d4*cosine(theta3dh)-a3*sine(theta3dh))-sine(theta2dh)*(d4*sine(theta3dh)+a3*cosine(theta3dh)+a2))

        sqrt=(h-numpy.power(f,2))

        theta=None

        if sqrt >=0 :
            theta=numpy.arctan2(f,self.ARM2*numpy.sqrt(sqrt))-numpy.arctan2(d4*sine_d(angles[2])+a3*cosine_d(angles[2])+a2,d4*cosine_d(angles[2])-a3*sine_d(angles[2]))

        return theta


    def calculate_thetha_3(self,end_pos,angles):

        a3=self.dh_table.get_paramf(3,'a')
        d4=self.dh_table.get_paramf(4,'d')
        a2=self.dh_table.get_paramf(2,'a')


        px=self.pos_in_matrix(end_pos,'p','x')
        py=self.pos_in_matrix(end_pos,'p','y')

        g214=cosine_d(angles[0])*px+sine_d(angles[0])*py

        g222=-self.pos_in_matrix(end_pos,'p','z')

        d=numpy.power(g214,2)+numpy.power(g222,2)-numpy.power(d4,2)-numpy.power(a3,2)-numpy.power(a2,2)

        e=(4*numpy.power(a2,2)*numpy.power(a3,2))+(4*numpy.power(a2,2)*numpy.power(d4,2))

        sqrt=(e-numpy.power(d,2))

        thetha=None

        if sqrt >= 0:

            thetha=numpy.arctan2(d,self.ARM*self.ELBOW*numpy.sqrt(sqrt))-numpy.arctan2(a3,d4)

        return thetha


    def move_robot_given_angles(self,angles):
        if len(angles)==self.dh_table.size():
            link=[0,1,2,3,4,5]
            self.robot.SetDOFValues(angles,link)


    def geometric_approach_top(self):

        angles=[0,0,0,0,0,0]

        for theta in range(0,6):
            angles[theta]= numpy.radians(float(raw_input("Theta"+ `(theta+1)` +"=")))

        t6=self.create_t6_matrix(angles)

        #self.robot.SetDOFValues(angles,[0,1,2,3,4,5])

        #t6=self.robot.GetLinks()[6].GetTransform()

        self.update_indicators(angles)

        inv=self.geometric_approach(t6)

        print "Original angles"
        print angles

        print "Calculated angles"
        print inv



    def geometric_approach(self,end_pos):
        angles=[0,0,0,0,0,0]

        angles[0]=self.thetha1_geometric(end_pos)

        angles[1]=self.thetha2_geometric(end_pos)

        angles[2]=self.theta3_geometric(end_pos)

        angles[3]=self.theta4_geometric(end_pos,angles)

        angles[4]=self.theta5_geometric(end_pos,angles)

        angles[5]=self.theta6_geometric(end_pos,angles)

        return angles


    def update_indicators(self,angles):

        d2=self.dh_table.get_paramf(2,'d')
        a2=self.dh_table.get_paramf(2,'a')
        d4=self.dh_table.get_paramf(4,'d')
        a3=self.dh_table.get_paramf(3,'a')

        c2=cosine_d(angles[1])
        c3=cosine_d(angles[2])
        s3=sine_d(angles[2])
        s2=sine_d(angles[2])

        s23=sine_d(angles[1]+angles[2])
        c23=cosine_d(angles[1]+angles[2])

        self.ARM=self.sign(-d4*s23-a3*c23-a2*c2)

        self.ELBOW=self.ARM*self.sign(d4*c3-a3*s3)

        self.ARM2=self.sign(c2*(d4*c3-a3*s3)-s2*(d4*s3+a3*c3+a2))

        t4=self.robot.GetLinks()[4].GetTransform()
        t6=self.robot.GetLinks()[6].GetTransform()

        z4=[]
        z4.append(self.pos_in_matrix(t4,'n','z'))
        z4.append(self.pos_in_matrix(t4,'s','z'))
        z4.append(self.pos_in_matrix(t4,'a','z'))

        s = []
        s.append(self.pos_in_matrix(t6, 's', 'x'))
        s.append(self.pos_in_matrix(t6, 's', 'y'))
        s.append(self.pos_in_matrix(t6, 's', 'z'))

        dot=numpy.dot(s,z4)

        if dot is 0:
            n=[]
            n.append(self.pos_in_matrix(t6,'n','x'))
            n.append(self.pos_in_matrix(t6,'n','y'))
            n.append(self.pos_in_matrix(t6,'n','z'))
            dot = numpy.dot(n,z4)

        self.WRIST= self.sign(dot)



    def thetha1_geometric(self,end_pos):
        px=self.pos_in_matrix(end_pos,'p','x')
        py=self.pos_in_matrix(end_pos,'p','y')
        d2=self.dh_table.get_paramf(2,'d')

        sqrt=numpy.power(px,2)+numpy.power(py,2)-numpy.power(d2,2)

        return numpy.arctan2(-self.ARM*py*numpy.sqrt(sqrt)-px*d2,-self.ARM*px*numpy.sqrt(sqrt)+py*d2)

    def thetha2_geometric(self,end_pos):
        K=self.ARM*self.ELBOW

        pz=self.pos_in_matrix(end_pos,'p','z')
        px=self.pos_in_matrix(end_pos,'p','x')
        py=self.pos_in_matrix(end_pos,'p','y')
        d2=self.dh_table.get_paramf(2,'d')
        a2=self.dh_table.get_paramf(2,'a')
        d4=self.dh_table.get_paramf(4,'d')
        a3=self.dh_table.get_paramf(3,'a')


        R=numpy.sqrt(numpy.power(px,2)+numpy.power(py,2)+numpy.power(pz,2)-numpy.power(d2,2))
        r=numpy.sqrt(numpy.power(px,2)+numpy.power(py,2))

        sina=-(pz/R)
        cosa=-((self.ARM*r)/R)

        cosb=((numpy.power(a2,2)+numpy.power(R,2)-(numpy.power(d4,2)+numpy.power(a3,2)))/(2*a2*R))

        sinb=numpy.sqrt(1-numpy.power(cosb,2))

        sint2=sina*cosb+(K)*cosa*sinb

        cost2=cosa*cosb-(K)*sina*sinb

        return numpy.arctan2(sint2,cost2)

    def theta3_geometric(self,end_pos):
        pz=self.pos_in_matrix(end_pos,'p','z')
        px=self.pos_in_matrix(end_pos,'p','x')
        py=self.pos_in_matrix(end_pos,'p','y')
        d2=self.dh_table.get_paramf(2,'d')
        a2=self.dh_table.get_paramf(2,'a')
        d4=self.dh_table.get_paramf(4,'d')
        a3=self.dh_table.get_paramf(3,'a')

        R=numpy.sqrt(numpy.power(px,2)+numpy.power(py,2)+numpy.power(pz,2)-numpy.power(d2,2))
        cosa=(numpy.power(a2,2)+(numpy.power(d4,2)+numpy.power(a3,2))-numpy.power(R,2))/(2*a2*numpy.sqrt(numpy.power(d4,2)+numpy.power(a3,2)))


        sina=self.ARM*self.ELBOW*numpy.sqrt(1-numpy.power(cosa,2))


        sinb=d4/(numpy.sqrt(numpy.power(d4,2)+numpy.power(a3,2)))

        cosb=numpy.abs(a3)/numpy.sqrt(numpy.power(d4,2)+numpy.power(a3,2))

        sint=sina*cosb - cosa*sinb
        cost=cosa*cosb + sina*sinb

        return numpy.arctan2(sint,cost)

    def theta4_geometric(self,end_pos,angles):

        az=self.pos_in_matrix(end_pos,'a','z')
        ax=self.pos_in_matrix(end_pos,'a','x')
        ay=self.pos_in_matrix(end_pos,'a','y')

        d2=self.dh_table.get_paramf(2,'d')
        a2=self.dh_table.get_paramf(2,'a')
        d4=self.dh_table.get_paramf(4,'d')
        a3=self.dh_table.get_paramf(3,'a')

        c1=cosine_d(angles[0])
        s1=sine_d(angles[0])
        c23=cosine_d(angles[1]+angles[2])
        s23=sine_d(angles[1]+angles[2])

        M=self.WRIST*1

        return numpy.arctan2(M*(c1*ay-s1*ax),M*(c1*c23*ax+s1*c23*ay+s23*az))

    def theta5_geometric(self,end_pos,angles):
        c1=cosine_d(angles[0])
        s1=sine_d(angles[0])
        c23=cosine_d(angles[1]+angles[2])
        s23=sine_d(angles[1]+angles[2])
        c4=cosine_d(angles[3])
        s4=sine_d(angles[3])

        az=self.pos_in_matrix(end_pos,'a','z')
        ax=self.pos_in_matrix(end_pos,'a','x')
        ay=self.pos_in_matrix(end_pos,'a','y')

        a=(c1*c23*c4 - s1*s4 )*ax + (s1*c23*c4 +c1*s4)*ay - s23*c4*az
        b=c1*s23*ax + s1*s23*ay + c23*az

        return numpy.arctan2(a,b)

    def theta6_geometric(self,end_pos,angles):
        c1=cosine_d(angles[0])
        s1=sine_d(angles[0])
        c23=cosine_d(angles[1]+angles[2])
        s23=sine_d(angles[1]+angles[2])
        c4=cosine_d(angles[3])
        s4=sine_d(angles[3])

        nz=self.pos_in_matrix(end_pos,'n','z')
        nx=self.pos_in_matrix(end_pos,'n','x')
        ny=self.pos_in_matrix(end_pos,'n','y')

        sz=self.pos_in_matrix(end_pos,'s','z')
        sx=self.pos_in_matrix(end_pos,'s','x')
        sy=self.pos_in_matrix(end_pos,'s','y')


        a=(-s1*c4 - c1*c23*s4)*nx + (c1*c4 - s1*c23*s4)*ny + (s23*s4)*nz
        b=(-s1*c4 - c1*c23*s4)*sx + (c1*c4 - s1*c23*s4)*sy + (s23*s4)*sz


        return numpy.arctan2(a,b)


def print_menu():

    out = False

    deci=None
    while out is False:

        print "--------MENU FOR ROBOTICS PROGRAM----------"
        print "|Enter the number for the action you want |"
        print "|0)Check inital matrix values             |"
        print "|1)Display all trans. matrices            |"
        print "|2)Forward kinematic movement             |"
        print "|3)Create File with points                |"
        print "|4)Inverse Kinematics-Circle Equation     |"
        print "|5)Inverse Kinematics-Tangent Half Angle  |"
        print "|6)Geometric approach                     |"
        print "|7)Exit                                   |"
        print "|___________SANTIAGO_VELEZ_SAFFON_________|"

        deci=int(raw_input("Option="))

        if deci in range(0,9):
            out=True

    return deci



def main():

    deci=print_menu()


    if deci is not 7:
        robot = Puma560('dh_table.xml')

    while deci is not 7:

        if deci is 2:
            robot.fordward_kinematics_checkings()

        if deci is 3:
            robot.fordward_kinematics_checkings(True)

        if deci is 4:
            robot.inverse_kinematics_top()

        if deci is 0:
            robot.test()

        if deci is 5:
            robot.inverse_kinematics_tangent_half_top()

        if deci is 6:
            robot.geometric_approach_top()

        deci=print_menu()


if __name__ == '__main__':
    main()