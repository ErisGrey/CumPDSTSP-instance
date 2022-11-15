

 /*#include <ilcplex/ilocplex.h>

#include <vector>
ILOSTLBEGIN

int
main()
{
    const float DRONESPEED = 40;
    const float BIGM = 9999;

    IloEnv   env;
    try {

        IloModel model(env, "chvatal");

        IloCplex cplex(model);
        cplex.setParam(IloCplex::TiLim, 1800);
        IloNum start = cplex.getTime();
        vector<float> Data;
        vector<float> Position;
        vector<vector<float>> Coordinate;
        
        ifstream infile("C:\\Users\\admin\\Downloads\\superDrone\\instances\\small\\15-r-e.csv");
        string str;
        while (!infile.eof()) {
            string ss;
            getline(infile, ss);
            ss += " ";
            str += ss;
        }
        stringstream ss(str);
        string Token;

        //Drone
        while (ss >> Token) {

            if (Token == "NUM_DRONES,")
            {
                ss >> Token;
                break;
            }
        }
        int NumDrone = atof(Token.c_str());
       // cout << NumDrone << endl;
        //Truck speed
        while (ss >> Token) {

            if (Token == "TRUCK_SPEED,")
            {
                ss >> Token;
                break;
            }
        }
        float TruckSpeed = atof(Token.c_str());
       // cout << TruckSpeed << endl;
        //Drone speed
        float DroneSpeed = DRONESPEED;
        
        //Nhap du lieu
        while (ss >> Token) {

            if (Token == "weight")
            {
                break;
            }

        }
        while (ss >> Token) {
            float f;
            f = atof(Token.c_str());
            Data.push_back(f);

        }
     
        
        //So node
        int Dimension = Data.size() / 4;
        cout << "Dimension: " << Dimension << endl;

        //Khoang cach
        vector <int> o;
        vector <int> p;
        for (int i = 0; i < Dimension; i++)
        {
            o.push_back(Data.at(4 * i + 1));
            p.push_back(Data.at(4 * i + 2));
            cout << "o[" << i << "] = " << o[i] << endl;
            cout << "p[" << i << "] = " << p[i] << endl;
        }
        IloArray<IloNumArray> Distance(env);
        for (int i = 0; i < Dimension; i++)
        {

            Distance.add(IloNumArray(env));
            for (int j = 0; j < Dimension; j++)
            {

                Distance[i].add(ceil(sqrt(pow(Data.at(4 * j + 1) - Data.at(4 * i + 1), 2) + pow(Data.at(4 * j + 2) - Data.at(4 * i + 2), 2))));
            }

        }

         for (int i = 0; i < Dimension; i++)
         {
             for (int j = 0; j < Dimension; j++)
             {
                  cout << "Distance[" << i << "][" << j << "] = " << Distance[i][j] << endl;
             }
         }  


         //Thoi gian cua xe
         IloArray<IloNumArray> TruckTime(env);
         for (int i = 0; i < Dimension; i++)
         {

             TruckTime.add(IloNumArray(env));
             for (int j = 0; j < Dimension; j++)
             {

                 TruckTime[i].add(Distance[i][j] / TruckSpeed);

             }

         }

         for (int i = 0; i < Dimension; i++)
         {
             for (int j = 0; j < Dimension; j++)
             {
                 cout << "TruckTime[" << i << "][" << j << "] = " << TruckTime[i][j] << endl;
             }
         }

         //Thoi gian cua Drone
         IloNumArray Dronetime(env, Dimension);
             for (int i = 0; i < Dimension; i++)
             {
                 Dronetime[i] = Distance[0][i] / DroneSpeed;
             }
             for (int i = 0; i < Dimension; i++)
             {
                 cout << "DroneTime[" << i << "] = " << Dronetime[i] << endl;
             }
        infile.close();
        


        IloArray<IloNumVarArray> x(env);
        for (int i = 0; i < Dimension; i++)
        {
            x.add(IloNumVarArray(env));
            for (int j = 0; j < Dimension; j++)
            {
                x[i].add(IloNumVar(env, 0, 1, ILOINT));
            }
        }


        IloArray<IloNumVarArray> y(env);
        for (int i = 0; i < Dimension; i++)
        {
            y.add(IloNumVarArray(env));
            for (int j = 0; j < Dimension; j++)
            {
                y[i].add(IloNumVar(env, 0, 1, ILOINT));
            }
        }


        IloArray<IloNumVarArray> v(env);
        for (int i = 0; i < Dimension; i++)
        {
            v.add(IloNumVarArray(env));
            for (int j = 0; j < Dimension; j++)
            {
                v[i].add(IloNumVar(env, 0.0, IloInfinity, ILOFLOAT));
            }
        }


        IloArray<IloNumVarArray> z(env);
        for (int i = 0; i < Dimension; i++)
        {
            z.add(IloNumVarArray(env));
            for (int j = 0; j < Dimension; j++)
            {
                z[i].add(IloNumVar(env, 0.0, IloInfinity, ILOFLOAT));
            }
        }

        IloNumVarArray u(env, Dimension, 1, Dimension - 1, ILOFLOAT);
       


        IloExpr exprSolution(env);
        for (int i = 0; i < Dimension; i++)
        {
            for (int j = 0; j < Dimension; j++)
            {
                exprSolution += v[i][j] + z[i][j];
            }
        }
        model.add(IloMinimize(env, exprSolution));

        IloExpr exprCondition1(env);
        for (int i = 1; i < Dimension; i++)
        {
            exprCondition1 += x[0][i];
        }
        model.add(exprCondition1 <= 1);

        IloExpr exprCondition2(env);
        for (int i = 1; i < Dimension; i++)
        {
            exprCondition2 += y[0][i];
        }
        model.add(exprCondition2 <= NumDrone);

       
        for (int j = 0; j < Dimension; j++)
        {
            IloExpr exprCondition3(env);
            for (int i = 0; i < Dimension; i++)
            {
                if (i != j)
                {
                    exprCondition3 += x[i][j] - x[j][i];
                }
            }
            model.add(exprCondition3 == 0);
        }

        for (int j = 0; j < Dimension; j++)
        {
            IloExpr exprCondition4(env);
            for (int i = 0; i < Dimension; i++)
            {
                if (i != j)
                {
                    exprCondition4 += x[i][j] + y[i][j];
                }
            }
            model.add(exprCondition4 == 1);
        }

        for (int i = 1; i < Dimension; i++)
        {
            for (int j = 1; j < Dimension; j++)
            {
                model.add(u[i] - u[j] + Dimension * x[i][j] <= Dimension - 1);
            }
        }

        for (int i = 1; i < Dimension; i++)
        {
            IloExpr exprCondition5(env);
            for (int j = 0; j < Dimension; j++)
            {
                if (j != i)
                {
                    exprCondition5 += v[j][i] + x[i][j] * TruckTime[i][j] - v[i][j];
                }
            }
            model.add(exprCondition5 == 0);

            model.add(v[0][i] - TruckTime[0][i] * x[0][i] == 0);
        }

        for (int i = 1; i < Dimension; i++)
        {
            IloExpr exprCondition6(env);
            for (int j = 0; j < Dimension; j++)
            {
                if (j != i)
                {
                    exprCondition6 += z[j][i] + y[i][j] * Dronetime[j] - z[i][j];
                }
            }
            model.add(exprCondition6 == 0);

            model.add(z[0][i] - Dronetime[i] * y[0][i] == 0);
        }

        for (int i = 0; i < Dimension; i++)
        {
            for (int j = 0; j < Dimension; j++)
            {
                if (i != j)
                {
                    model.add(v[i][j] >= TruckTime[i][j] * x[i][j]);
                    model.add(v[i][j] <= BIGM * x[i][j]);
                    model.add(z[i][j] >= Dronetime[j] * y[i][j]);
                    model.add(z[i][j] <= BIGM * y[i][j]);
                }
            }
        }


        cplex.solve();
        
        
        cplex.out() << "Solution status " << cplex.getStatus() << endl;
        cplex.out() << "Objective value " << cplex.getObjValue() << endl;

        cplex.out() << "Time:  " << cplex.getTime() - start << endl; 

        cplex.exportModel("lpex3.lp"); 
    }
    catch (IloException& e) {
        cerr << "Concert exception caught: " << e << endl;
    }
    catch (...) {
        cerr << "Unknown exception caught" << endl;
    }
    
    env.end();  
    return 0; 
}  */
