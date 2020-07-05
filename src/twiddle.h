#ifndef TWIDDLE_H
#define TWIDDLE_H

vector<int> twiddle(int tol)
{
    vector<int> p = [0, 0, 0];
    vector<int> dp = [1, 1, 1];
    x_trajectory, y_trajectory, best_err = run(robot, p)
    it = 0
    while (sum(dp) > tol)
    {
        std::cout << "Iteration " << it << " best error = " << best_err << std::endl;
        for (int i = 0; i < p.size(); i++)
        {
            p[i] += dp[i];
            
            robot = make_robot()
            x_trajectory, y_trajectory, err = run(robot, p)
            
            if (err < best_err)
            {
                best_err = err;
                dp[i] *= 1.1;
            }
            else
            {
                p[i] -= 2*dp[i];
                
                robot = make_robot()
                x_trajectory, y_trajectory, err = run(robot, p)

                if (err < best_err)
                {
                    best_err = err;
                    dp[i] *= 1.1;
                }
                else
                {
                    p[i] += dp[i];
                    dp[i] *= 0.9;
                }
            }
        }
        it += 1;
    }
    std::cout << "Final params: P = " << p[0] << ", D = " <<
                    p[1] << ", I = " << p[2] << std::endl;
    std::cout << "Best error = " << best_err << std::endl;
    return p;
}
#endif  // TWIDDLE_H