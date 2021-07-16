
import java.io.*;
import java.util.*;

import ilog.concert.*;
import ilog.cplex.IloCplex;

public class END507_project{
	
	public static int M = 9;

	public static void main(String[] args) {
		
		routingModel();
		
	}
	
//	public static void model1() { //örnek model
//		
//		try {
//			IloCplex cplex = new IloCplex();
//			
//			//variables
//			IloNumVar x = cplex.numVar(0, Double.MAX_VALUE, "x");
//			IloNumVar y = cplex.numVar(0, Double.MAX_VALUE, "y");
//			
//			//expression
//			IloLinearNumExpr obj = cplex.linearNumExpr();
//			obj.addTerm(0.12, x);
//			obj.addTerm(0.15, y);
//			
//			// define objective
//			cplex.addMinimize(obj);
//			
//			//define constraints
//			cplex.addGe(cplex.sum(cplex.prod(60, x),cplex.prod(60, y)), 300);
//			cplex.addGe(cplex.sum(cplex.prod(12, x),cplex.prod(6, y)), 36);
//			cplex.addGe(cplex.sum(cplex.prod(10, x),cplex.prod(30, y)), 90);
//			
//			//solve
//			if(cplex.solve()) {
//				System.out.println("obj 	= "+cplex.getObjValue());
//				System.out.println("x		= "+cplex.getValue(x));
//				System.out.println("y		= "+cplex.getValue(y));
//			}
//			else {
//				System.out.println("Model not solved!");
//			}
//		}
//		
//		catch(IloException exc){
//			exc.printStackTrace();
//		}
//	}
	
	public static void routingModel() {
		
		//sets
		int numberOfIncidents = 1;
		int numberOfInitialLocation = 1;
		int numberOfNodes = numberOfIncidents+numberOfInitialLocation;
		int numberOfVehicles = 1;
		int numberOfArcs = 1;
		
		try {
			IloCplex cplex = new IloCplex();
			//parameters
			double[] occurenceTime = new double[numberOfIncidents];
			double[] serviceTime = new double[numberOfIncidents];
			double[][] travelTimeBtwNode = new double[numberOfNodes][numberOfNodes];
			double[][] travelTimeToArc = new double[numberOfIncidents][numberOfArcs];
			int[][] beta = new int[numberOfIncidents][numberOfVehicles];
			int[][] teta = new int[numberOfNodes][numberOfVehicles];
			
			//baby parameters
			travelTimeBtwNode[0][0]=1;
			travelTimeBtwNode[0][1]=1;
			travelTimeBtwNode[1][0]=1;
			travelTimeBtwNode[1][1]=1;
			
			travelTimeToArc[0][0]=0.5;
			
			beta[0][0]=1;
			teta[0][0]=1;
			teta[1][0]=0;
			
			//end-parameters
			
			//decision variables
			IloIntVar[][] y = new IloIntVar[numberOfIncidents][numberOfVehicles];
			ArrayList<IloIntVar[][]> x = new ArrayList<IloIntVar[][]>();
			IloNumVar[][] arrivalTime = new IloNumVar[numberOfNodes][numberOfVehicles];
			IloNumVar[][] departureTime = new IloNumVar[numberOfNodes][numberOfVehicles];
			IloNumVar[] responseTime = new IloNumVar[numberOfIncidents];
			IloNumVar[][] alpha = new IloNumVar[numberOfIncidents][numberOfArcs];
			
			for(int k = 0; k<numberOfVehicles;k++) {
				x.add(new IloIntVar[numberOfNodes][numberOfNodes]);
				for(int i = 0; i<numberOfNodes;i++) {
					for(int j = 0; j<numberOfNodes;j++) {
						x.get(k)[i][j] = cplex.intVar(0, 1);
					}
				}
			}
			
			for(int i = 0; i<numberOfIncidents;i++) {
				responseTime[i] = cplex.numVar(0, Double.MAX_VALUE);
				for(int k = 0; k<numberOfVehicles;k++) {
					y[i][k] = cplex.intVar(0, 1);
				}
			}
			
			for(int i = 0; i<numberOfNodes;i++) {
				for(int k = 0; k<numberOfVehicles;k++) {
					arrivalTime[i][k] = cplex.numVar(0, Double.MAX_VALUE);
					departureTime[i][k] = cplex.numVar(0, Double.MAX_VALUE);
				}
			}
			
			for(int a = 0; a<numberOfArcs; a++) {
				for(int i = 0; i<numberOfIncidents;i++) {
					alpha[i][a] = cplex.numVar(0, Double.MAX_VALUE);
				}
			}
			//end-decision variables
			
			//objective---------------------------------------------------------------
			IloLinearNumExpr obj = cplex.linearNumExpr();
			
			for(int a = 0; a<numberOfArcs; a++) {
				for(int i = 0; i<numberOfIncidents;i++) {
					obj.addTerm(1, alpha[i][a]);
				}
			}
			
			cplex.addMinimize(obj);
			//end-objective-----------------------------------------------------------
			
			//constraints
			
			//constraint 2
			
			for(int a = 0; a<numberOfArcs; a++) {
				for(int i = 0; i<numberOfIncidents;i++) {
					IloLinearNumExpr constraint2 = cplex.linearNumExpr();
					constraint2.addTerm(1, alpha[i][a]);
					constraint2.addTerm(-1/travelTimeToArc[i][a], responseTime[i]);
					cplex.addEq(constraint2, -occurenceTime[i]/travelTimeToArc[i][a]);
				}
			}
			
			//constraint 3
			
			for(int i = 0; i<numberOfIncidents;i++) {
				IloLinearNumExpr constraint3 = cplex.linearNumExpr();
				for(int k = 0; k<numberOfVehicles;k++) {					
					constraint3.addTerm(beta[i][k], y[i][k]);
				}
				cplex.addEq(constraint3, 1);
			}
			
			//constraint 4 & 5
			
			for(int k = 0; k<numberOfVehicles;k++) {
				for(int i = 0; i<numberOfInitialLocation;i++) {
					IloLinearNumExpr constraint4 = cplex.linearNumExpr();
					IloLinearNumExpr constraint5 = cplex.linearNumExpr();
					for(int j = 0; j<numberOfNodes && !(i==j);j++) {
						constraint4.addTerm(1, x.get(k)[i][j]);
						constraint5.addTerm(1, x.get(k)[j][i]);
					}
					cplex.addLe(constraint4, M*teta[i][k]);
					cplex.addLe(constraint5, M*teta[i][k]);
				}
			}
			
			//constraint 6
			
			for(int k = 0; k<numberOfVehicles;k++) {
				for(int i = 0; i<numberOfNodes;i++) {
					IloLinearNumExpr constraint6 = cplex.linearNumExpr();
					for(int j = 0; j<numberOfNodes && !(i==j);j++) {
						constraint6.addTerm(1, x.get(k)[i][j]);
						constraint6.addTerm(-1, x.get(k)[j][i]);
					}
					cplex.addLe(constraint6, 0);
				}
			}
			
			//constraint 7 & 8
			
			for(int k = 0; k<numberOfVehicles;k++) {
				for(int i = 0; i<numberOfIncidents;i++) {
					IloLinearNumExpr constraint7 = cplex.linearNumExpr();
					IloLinearNumExpr constraint8 = cplex.linearNumExpr();
					for(int j = 0; j<numberOfNodes && !(i==j);j++) {
						constraint7.addTerm(1, x.get(k)[i][j]);
						constraint7.addTerm(-1, y[i][k]);
						constraint8.addTerm(1, x.get(k)[j][i]);
						constraint8.addTerm(-1, y[i][k]);
					}
					cplex.addLe(constraint7, 0);
					cplex.addLe(constraint8, 0);
				}
			}
			
			//constraint 9 & 10
			
			for(int k = 0; k<numberOfVehicles;k++) {
				for(int i = 0; i<numberOfNodes;i++) {
					for(int j = 0; j<numberOfNodes && !(i==j);j++) {
						IloLinearNumExpr constraint9 = cplex.linearNumExpr();
						IloLinearNumExpr constraint10 = cplex.linearNumExpr();
						
						constraint9.addTerm(1, departureTime[i][k]);
						constraint9.addTerm(-1, arrivalTime[i][k]);
						constraint9.addTerm(M, x.get(k)[i][j]);
						constraint10.addTerm(1, departureTime[i][k]);
						constraint10.addTerm(-1, arrivalTime[i][k]);
						constraint10.addTerm(-M, x.get(k)[i][j]);
						
						cplex.addLe(constraint9, -travelTimeBtwNode[i][j]+M);
						cplex.addGe(constraint10, -travelTimeBtwNode[i][j]-M);
					}
				}
			}
			
			//constraint 11
			for(int i = 0; i<numberOfIncidents;i++) {
				for(int k = 0; k<numberOfVehicles;k++) {	
					IloLinearNumExpr constraint11 = cplex.linearNumExpr();
					constraint11.addTerm(1, departureTime[i][k]);
					constraint11.addTerm(-1, arrivalTime[i][k]);
					cplex.addGe(constraint11, 0);
				}
			}
			
			//constraint 12
			for(int i = 0; i<numberOfIncidents;i++) {
				for(int k = 0; k<numberOfVehicles;k++) {	
					IloLinearNumExpr constraint12 = cplex.linearNumExpr();
					constraint12.addTerm(1, departureTime[i][k]);
					constraint12.addTerm(-1, arrivalTime[i][k]);
					constraint12.addTerm(-serviceTime[i], y[i][k]);
					cplex.addEq(constraint12, 0);
				}
			}
			
			//constraint 13&14 - eðer çalýþmazsa i'yi I üzerinden yaz
			for(int i = 0; i<numberOfIncidents;i++) {
				for(int k = 0; k<numberOfVehicles;k++) {	
					IloLinearNumExpr constraint13 = cplex.linearNumExpr();
					IloLinearNumExpr constraint14 = cplex.linearNumExpr();
					constraint13.addTerm(1, arrivalTime[i][k]);
					constraint13.addTerm(-M, y[i][k]);
					constraint14.addTerm(1, departureTime[i][k]);
					constraint14.addTerm(-M, y[i][k]);
					cplex.addLe(constraint13, 0);
					cplex.addLe(constraint14, 0);
				}
			}
			
			//constraint 15
			for(int k = 0; k<numberOfVehicles;k++) {
				for(int i = 0; i<numberOfIncidents;i++) {
					for(int j = 0; j<numberOfNodes && !(i==j);j++) {
						IloLinearNumExpr constraint15 = cplex.linearNumExpr();
						constraint15.addTerm(occurenceTime[i], y[i][k]);
						constraint15.addTerm(travelTimeBtwNode[i][j], x.get(k)[j][i]);
						constraint15.addTerm(-1, arrivalTime[i][k]);
						cplex.addLe(constraint15, 0);
					}
				}
			}
			
			//constraint 16
			for(int i = 0; i<numberOfIncidents;i++) {
				for(int k = 0; k<numberOfVehicles;k++) {	
					IloLinearNumExpr constraint16 = cplex.linearNumExpr();
					constraint16.addTerm(1, responseTime[i]);
					constraint16.addTerm(-1, arrivalTime[i][k]);
					cplex.addGe(constraint16, 0);
				}
			}
			
			//end-constraints
			
			//solve
			if(cplex.solve()) {
				System.out.println("obj 	= "+cplex.getObjValue());
				System.out.println("y 	= "+cplex.getValue(y[0][0]));
				System.out.println("x 	= "+cplex.getValue(x.get(0)[0][0]));
				
//				for(int k = 0; k<numberOfVehicles;k++) {
//					System.out.println("x");
//					for(int i = 0;i<numberOfNodes;i++) {
//						System.out.println("x");
//						for(int j = 0; j<numberOfNodes;j++) {
//							System.out.println("x");
//							int a = (int) cplex.getValue(x.get(k)[i][j]);
//							System.out.println("A"+a);
//						}
//					}
//				}
			}
			else {
				System.out.println("Model not solved!");
			}
			cplex.close();
		}
		
		catch(IloException exc) {
			
		}
		
		
		
	}
	

}
