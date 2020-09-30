from Tkinter import*
rootwindow = Tk()
name=Label(rootwindow, text="Name")
name.grid(column=0,row=1)
name_entry=Entry(rootwindow)
name_entry.grid(column=1, row=1)
tel=Label(rootwindow, text="Tel")
tel.grid(column=0,row=2)
tel_entry=Entry(rootwindow)
tel_entry.grid(column=1, row=2)
pic=Label(rootwindow, text="Photo")
pic.grid(column=0,row=3)
pic_entry=Entry(rootwindow)
pic_entry.grid(column=1, row=3)
               

con={}
l=[]
o = open('CONTACTS.txt','r')
cont=o.readlines()
for i in cont:
    i=i.split()
    con[i[0]]=[i[1],i[2]]

def add_number():
    names=name_entry.get()
    phones=tel_entry.get()
    pics=pic_entry.get()
    LB.insert(END,names)
    name_entry.delete(0, END)
    tel_entry.delete(0, END)
    pic_entry.delete(0, END)
    con[names]=[phones,pics]
Add=Button(rootwindow, text="Add", bg="blue", fg="orange", command = add_number)
Add.grid(column=3, row=3)
LB=Listbox(rootwindow)
LB.grid(column=1, row=4)
pic_label=Button(rootwindow)    
pic_label.grid(column=3, row=6) 
for i in con:
    LB.insert(END,i)
def loadit():
    x=LB.get(ACTIVE)
    photo=PhotoImage(file=con[LB.get(ACTIVE)][1])    
    name_label=Label(rootwindow, text=x)
    name_label.grid(column=3, row=4)
    tel_label=Label(rootwindow, text=con[x][0])
    tel_label.grid(column=3, row=5)
    pic_label.config(image=photo)
    pic_label.image=photo
    
def delete_D():
    del con[LB.get(ANCHOR)]    
    LB.delete(ANCHOR)
delete=Button(rootwindow, text="Delete", bg="blue", fg="orange", command = delete_D)
delete.grid(column=1, row=5)
load=Button(rootwindow, text="Load", bg="blue", fg="orange", command = loadit)
load.grid(column=2, row=5)
new={}
l=""
def save():
    w=open('CONTACTS.txt','w')
    
    for i in con:
        w.write(i+' '+con[i][0]+' '+con[i][1]+"\n")
   
        
    rootwindow.destroy()
    
rootwindow.protocol("WM_DELETE_WINDOW",save)
rootwindow.mainloop()