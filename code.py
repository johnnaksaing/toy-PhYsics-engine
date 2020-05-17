# Now Doing 
# Floor colision(Ground Contact)
#   doing!
#
# To-Do
# Calculation method updating
#
# 룽게-쿠타 방법 << ??
# Midpoint Method
# 조절

import glfw
from OpenGL.GL import *
import numpy as np

# glTranslate3f(P.x, P.y, P.z = 0);
# glTranslate3fv(np.array([x,y,0.]))

# glRotate3f(angle, A.x, A.y, A.z = 0);
# rotate 마찬가지

gEditingPoint = ''
t_old = 0.
windowSize = 800

class worldSetting:
    def __init__(self,k,c,g,L,epsilon,method,e):
        self.k = float(k)               # spring 탄성계수
        self.c = float(c)               # damping term
        self.g = float(g)               # gravity force
        self.L = float(L)               # spring original length
        self.epsilon = float(epsilon)   # collision detection threshold
        self.e = float(e)               # 바닥 반발계수
        self.method = method            # "euler" | "midpoint" | "runge-kutta"
        

class Point:
    def __init__(self,x,y,m,name):
        self.location = np.array([x,y,0.])
        self.speed = np.array([0.,1.,0.])
        self.mass = m
        self.name = name
    def __str__(self):
        return "P"+str(self.name)


#P = Point(X, Y, Mass, "Name")
# "Name"은 INT
P0 = Point(100.,200.,10.,0)
P1 = Point(200.,300.,10.,1)
P2 = Point(300.,150.,10.,2)
P3 = Point(400.,300.,10.,3)



# TODO :: Point 30개 이상 추가
P4 = Point(430.,400.,10.,4)
P5 = Point(450.,420.,10.,5)
P6 = Point(450.,430.,10.,6)
P7 = Point(450.,440.,10.,7)

P8 = Point(420.,420.,10.,8)
P9 = Point(410.,410.,10.,9)

Points = [P0,P1,P2,P3,P4,P5,P6,P7]


# TODO 4월 25일:
# threshold using epsilon
# during colidion 처리
# https://www.cs.cmu.edu/~baraff/sigcourse/notesc.pdf
# pdf 7절
def is_collide(P,epslion):
    # (X-P)*N is > 0 : inside, = 0 : collide, < 0 : outside
    N_plain = np.array([0.,-1.,0.])
    point = np.array([100., 20., 0])

    #print(np.dot(P.location - point,N_plain))

    if np.dot((P.location - point),N_plain) > epslion:
        return "outside"
    elif epslion >= np.dot((P.location - point),N_plain) > -epslion:
        return "ing"
    else: 
        return "inside"
        

# (X-P)*N < e,
# N * V < 0
def collide_detection(P,epsilon,flag):

    N_plain = np.array([0.,1.,0.])
    point = np.array([20., 20., 0.])

    # 참고 : while colliding -> Force 처리
    if flag != "Force":
        if np.dot((P.location - point), N_plain) < epsilon and np.dot(N_plain,P.speed) < 0.:
            return True
        else:
            return False
    else:
        if np.abs(np.dot((P.location - point), N_plain)) < epsilon and np.abs(np.dot(N_plain,P.speed)) < epsilon :
            return True
        else:
            return False



def force(P, k, c, g, L, epslion):
    
    Force = np.array([0.,0.])
    
    # list: 중력, 장력, 충돌 처리, 마찰력(아마도)

    # gravity
    Force[1] = -g*P.mass #-9800. 정도가 괜찮았음

    # spring force
        # 실로 엮인 점 두개 구함

        # better approach:
        # for P in Points:
    for point in range(len(Points)):
        if (P == Points[point]):
            # 예외 처리 필요: 0, 1, len(Points)-1 : 끝            
            if point == 0:
                PL = Points[-1]; PR = Points[1]; PT = Points[-2]
            elif point == 1:
                PL = Points[0]; PR = Points[2]; PT = Points[-1]
            elif point == len(Points)-1:
                PL = Points[point-1]; PR = Points[0]; PT = Points[point-2]
            else:
                PL = Points[point-1]; PR = Points[point+1]; PT = Points[point-2]                
            


        # 실 길이 구함
    # 참고 : np.linalg.norm(a-b)는 길이 크기
    # 내가필요한건 벡터
    # https://www.it-swarm.dev/ko/python/%EC%9C%A0%ED%81%B4%EB%A6%AC%EB%93%9C-%EA%B1%B0%EB%A6%AC%EB%A5%BC-numpy%EB%A1%9C-%EC%96%B4%EB%96%BB%EA%B2%8C-%EA%B3%84%EC%82%B0%ED%95%A0-%EC%88%98-%EC%9E%88%EC%8A%B5%EB%8B%88%EA%B9%8C/967035388/
    PL_length = np.linalg.norm(P.location - PL.location)
    PR_length = np.linalg.norm(P.location - PR.location)
    PT_length = np.linalg.norm(P.location - PT.location)
    
    PL_direction = (P.location - PL.location) / PL_length
    PR_direction = (P.location - PR.location) / PR_length
    PT_direction = (P.location - PT.location) / PT_length

        # 실 원래길이 L

        # 실 길이 차이만큼 공식 적용
    PL_strength = -(k * (PL_direction) * (PL_length - L)) - c * P.speed  #-(PL_length * k) * PL_dist # / 벡터 크기     # + -k * PL_length * PL_dist ** 2
    PR_strength = -(k * (PR_direction) * (PR_length - L)) - c * P.speed  # * PL_Length
    PT_strength = -(k * (PT_direction) * (PT_length - L)) - c * P.speed

        # Force에 더해줌
    Force += PL_strength[0:2] + PR_strength[0:2] + PT_strength[0:2]
    
    # Colision Response 
    if collide_detection(P,epslion,"Force"):
        # 평면에 대한 Collision 처리 중...
        N = np.array([0., 1., 0.])

        Fn = np.dot(Force, N)*Force # Force
        Ft = Force - Fn
            
        Force = Ft
        

    return Force

        #TODO? Friction


# [Xnew, Ynew] = [Xold, Yold] + dt * Vold
# Vnew = Vold + dt * Force/mass
def simulate(k,c,g,L,epslion, method, e):
    
    global P0,P1,P2,P3,gEditingPoint, t_old
    t = glfw.get_time()
    dt = t - t_old

    for P in Points:
        
        location_old = np.array([P.location[0], P.location[1]])
        velocity_old = np.array([P.speed[0] ,     P.speed[1]])

        # Collision Response
        if collide_detection(P,epslion,""):
            N = np.array([0., 1., 0.])

            Vn = np.dot(P.speed, N)*P.speed #P.speed
            Vt = P.speed - Vn
            
            # e는 반발계수
            P.speed = Vt - e * Vn


        ###########
        # METHOD1 # : Euler
        ###########
        if method == "euler":
            location_new = [
                location_old[0] + dt * velocity_old[0],
                location_old[1] + dt * velocity_old[1]
            ]
            velocity_new = velocity_old   +  dt * force(P, k, c, g, L, epslion)/P.mass

        ###########
        # METHOD2 # : Midpoint
        ###########
        elif method == "midpoint":

            # 우선 euler method로 진행한다.
            location_new = [
                location_old[0] + dt * velocity_old[0],
                location_old[1] + dt * velocity_old[1]
            ]
            velocity_new = velocity_old   +  dt * force(P, k, c, g, L, epslion)/P.mass

            # 중간 부분을 구한다.
            location_mid = (location_old + location_new) * 0.5
            velocity_mid = (velocity_old + velocity_new) * 0.5

            P.location[0] = location_mid[0]
            P.location[1] = location_mid[1]
            P.speed[0] = velocity_mid[0]
            P.speed[1] = velocity_mid[1]

            # 중간 부분에서 진행한다.
            location_new = location_mid + dt * velocity_mid * 0.5
            velocity_new = velocity_mid + dt * force(P, k, c, g, L, epslion)/P.mass * 0.5



        ###########
        # METHOD3 # : Runge-kutta
        ###########

        # location_new = location_old + (k1 + 2*k2 + 2*k3 + k4)/6
        elif method == "runge-kutta":

            # 우선 euler method로 진행한다.
            location_new = [
                location_old[0] + dt * velocity_old[0],
                location_old[1] + dt * velocity_old[1]
            ]
            velocity_new = velocity_old   +  dt * force(P, k, c, g, L, epslion)/P.mass
            
            # 중간 부분을 구한다.
            location_mid = (location_old + location_new) * 0.5
            velocity_mid = (velocity_old + velocity_new) * 0.5
            
            h = dt

            # h/2만큼 진행시켰을 때 상태 : temp
            temp = P

            kx1 = location_old
            kv1 = velocity_old
            temp.location[0] = kx1[0]
            temp.location[1] = kx1[1]
            temp.speed[0] = kv1[0]
            temp.speed[1] = kv1[1]
            
            kx2 = kx1 + 0.5 * h * kv1
            kv2 = kv1 + 0.5 * h * force(temp,k,c,g,L,epslion)/temp.mass
            temp.location[0] = kx2[0]
            temp.location[1] = kx2[1]
            temp.speed[0] = kv2[0]
            temp.speed[1] = kv2[1]

            kx3 = kx2 + 0.5 * h * kv2
            kv3 = kv2 + 0.5 * h * force(temp,k,c,g,L,epslion)/temp.mass
            temp.location[0] = kx3[0]
            temp.location[1] = kx3[1]
            temp.speed[0] = kv3[0]
            temp.speed[1] = kv3[1]

            kx4 = kx3 + h * kv3
            kv4 = kv3 + h * force(temp,k,c,g,L,epslion)/temp.mass

            dx = h * (kx1 + 2. * kx2 + 2. * kx3 + kx4) / 6.
            dv = h * (kv1 + 2. * kv2 + 2. * kv3 + kv4) / 6.

            location_new =  location_old + dx
            velocity_new = velocity_old + dv



        P.location[0] = location_new[0]
        P.location[1] = location_new[1]
        P.speed[0] = velocity_new[0]
        P.speed[1] = velocity_new[1]

        # 반영
        #location_old = location_new
        #velocity_old = velocity_new

        

    #simulation ends here
    t_old = t
    

def render():
    global P0, P1, P2, P3, windowSize
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
    glEnable(GL_DEPTH_TEST)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    glOrtho(0,windowSize, 0,windowSize, -1, 1)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

    glColor3ub(0, 255, 0)

    # Lines
        # 이웃(PL, PR 관계)
    for i in range(len(Points)):
        glBegin(GL_LINE_STRIP)
        p1 = Points[i-1]; p2 = Points[i]
        for t in np.arange(0,1,.07):
            p = (1-t)*p1.location + t*p2.location
            glVertex3fv(p)
        glEnd()
    
        # 하나 건너 이웃(PT 관계)
    for i in range(len(Points)):
        glBegin(GL_LINE_STRIP)
        p1 = Points[i-2]; p2 = Points[i]
        for t in np.arange(0,1,.07):
            p = (1-t)*p1.location + t*p2.location
            glVertex3fv(p)
        glEnd()

    '''
    glBegin(GL_LINE_STRIP)
    for t in np.arange(0,1,.01):
        p = (1-t)*P0.location + t*P1.location
        glVertex3fv(p)
    glEnd()
    '''
    # Points
    glPointSize(20.)
    glBegin(GL_POINTS)
    for P in Points:
        glVertex3fv(P.location)
    glEnd()


    #Floor
    glColor3ub(255, 255, 0)
    glBegin(GL_LINE_STRIP)
    glVertex3fv(np.array([10.,20.,0.]))
    glVertex3fv(np.array([float(windowSize),20.,0.]))
    glEnd()

    #Wall
    glBegin(GL_LINE_STRIP)
    glVertex3fv(np.array([10.,20.,0.]))
    glVertex3fv(np.array([10.,200.,0.]))
    glVertex3fv(np.array([0., 200.,0.]))
    glEnd()
    


def button_callback(window, button, action, mod):
    global Points, P0, P1, P2, P3, gEditingPoint, windowSize
    if button==glfw.MOUSE_BUTTON_LEFT:
        x, y = glfw.get_cursor_pos(window)
        y = windowSize - y
        if action==glfw.PRESS:

            for P in Points:
                if np.abs(x-P.location[0])<10 and np.abs(y-P.location[1])<10:
                    gEditingPoint = str(P)

        elif action==glfw.RELEASE:
            gEditingPoint = ''

def cursor_callback(window, xpos, ypos):
    global Points, P0, P1, gEditingPoint, windowSize
    ypos = windowSize - ypos
    for P in Points:
        if gEditingPoint == str(P):
            P.location[0]=xpos; P.location[1]=ypos

def main():
    global windowSize

    if not glfw.init():
        return

    # TODO :: 이거 입력 고치는법
    k = 250.                 # spring 탄성계수
    c = 100.                 # damping
    g = 9800.                # gravity force
    L = 150.                 # spring의 원래 length
    epsilon = 0.5            # collision detection threshold
    method = "euler"      # euler | midpoint | runge-kutta
    e = 0.02                  # 바닥 반발계수

    window = glfw.create_window(windowSize,windowSize,'PA2', None,None)
    if not window:
        # TODO:: input refresh 여기서
        k,c,g,L,epsilon,method,e = input("k c g L epsilon method e").split()
        world = worldSetting(k,c,g,L,epsilon,method,e)
        glfw.terminate()
        return


    glfw.make_context_current(window)
    glfw.set_mouse_button_callback(window, button_callback)
    glfw.set_cursor_pos_callback(window, cursor_callback)
    
    # t 설정 하는법??
    glfw.swap_interval(1)

    while not glfw.window_should_close(window):
        glfw.poll_events()
        simulate(k,c,g,L,epsilon,method,e)
        render()
        glfw.swap_buffers(window)
    glfw.terminate()

if __name__ == "__main__":
    main()
