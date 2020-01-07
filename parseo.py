import re 

class Parseo:
	def leer_archivo(self,nombre):
		with open(nombre,"r") as a:
			return a.read()

	def parseo(self,nombre):
		s= self.leer_archivo(nombre)
		ll= s.split('\n')
		
		acciones ={}
		parametro = {}
		acciones['action']=[]
		parametro['parametro'] = []
		patron = ('action:')
		pal = re.compile(patron)
	
		for i in range(len(ll)):
			q = ll[i]
			j= q.find(" parameters: ")
			if j >= 0:
					t = q.split(":")
					parametro['parametro'].append(t[1]) 
			if pal.match(ll[i]) != None:
				t= ll[i].split(': ')
				acciones['action'].append(t[1])
						
		#print acciones
		#print len(acciones)
		#print parametro
		#print parametro['parametro'][0]
		patron1 = re.compile(r'\w\w+')
		para = []
		
		for i in range(len(acciones['action'])):
			qq = parametro['parametro'][i]
			
			palabra = patron1.findall(qq)
			para.append(palabra)
			print palabra
			#print len(palabra)
		#print para[0][0]
		tt = len(acciones['action']) / 2
		miti = []
		miti2 = []
		param = []
		param2 = []
		for i in range (tt):
			miti.append(acciones['action'][i])
			param.append(para[i])
		for i in range (tt,len(acciones['action'])):
			miti2.append(acciones['action'][i])
			param2.append(para[i])
		#print miti , param
		#print miti2, param2
		accion = {}
		accion['action'] = []
		parame = []
		for i in range(len(acciones['action'])):
			accion['action'].append('s')
			parame.append('s')
		for i in range(len(acciones['action'])):
			if i % 2 == 0 :
				accion['action'][i] = miti[0]
				parame[i] = param[0]
				miti.pop(0)
				param.pop(0)
			else:
				accion['action'][i] = miti2[0]
				parame[i] = param2[0]
				miti2.pop(0)
				param2.pop(0)
		print accion['action']
		for i in range(len(acciones['action'])):
			acciones['action'][i] = accion['action'][i]
			para[i] = parame[i]
		return acciones, para 


def main():
		par = Parseo()
		rt , t = par.parseo('elplan.txt')
		print "acciones ", rt
		print "parametros", t
if __name__ == '__main__':
	main()