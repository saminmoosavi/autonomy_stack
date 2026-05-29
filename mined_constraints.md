# Mined constraints - SCAND social-compliance envelope

Program: `openevolve_output_scand/best/best_program.py`

Formula:
```
G(speed <= 2.603) & G(yaw_rate <= 3.293) & G(accel <= 3.213) & G(jerk <= 5.862) & G(lat_accel <= 2.228) & G(cmd_steer <= 1.000) & G(min_clearance >= 0.483) & G(crowd_count >= 3 | min_clearance >= 0.6) & G(front_clearance >= 0.505) & G(left_clearance >= 0.500) & G(right_clearance >= 0.500) & G(rear_clearance >= 0.483) & G(ttc >= 0.452) & G(ped_approach_rate <= 5.000) & G(ped_clearance >= 0.500) & G(front_ped_clearance >= 0.500) & G(ped_ttc >= 0.422)
```

combined_score 0.890 | balanced_accuracy 0.856 | tightness 0.997

## 17 constraint(s) - the robot must, at all times:

1. **forward speed must stay at or below 2.603 m/s.**
   `G(speed <= 2.603)`

2. **turn rate must stay at or below 3.293 rad/s.**
   `G(yaw_rate <= 3.293)`

3. **longitudinal acceleration must stay at or below 3.213 m/s^2.**
   `G(accel <= 3.213)`

4. **jerk must stay at or below 5.862 m/s^3.**
   `G(jerk <= 5.862)`

5. **lateral acceleration must stay at or below 2.228 m/s^2.**
   `G(lat_accel <= 2.228)`

6. **sustained steering effort must stay at or below 1.**
   `G(cmd_steer <= 1)`

7. **clearance from the nearest obstacle must stay at or above 0.483 m.**
   `G(min_clearance >= 0.483)`

8. **unless number of nearby obstacles is at least 3, clearance from the nearest obstacle must stay at or above 0.6 m.**
   `G(crowd_count >= 3 | min_clearance >= 0.6)`

9. **clearance ahead must stay at or above 0.505 m.**
   `G(front_clearance >= 0.505)`

10. **clearance to the left must stay at or above 0.5 m.**
   `G(left_clearance >= 0.5)`

11. **clearance to the right must stay at or above 0.5 m.**
   `G(right_clearance >= 0.5)`

12. **clearance behind must stay at or above 0.483 m.**
   `G(rear_clearance >= 0.483)`

13. **time-to-collision ahead must stay at or above 0.452 s.**
   `G(ttc >= 0.452)`

14. **approach speed toward the nearest pedestrian must stay at or below 5 m/s.**
   `G(ped_approach_rate <= 5)`

15. **clearance from the nearest pedestrian must stay at or above 0.5 m.**
   `G(ped_clearance >= 0.5)`

16. **clearance to the nearest pedestrian ahead must stay at or above 0.5 m.**
   `G(front_ped_clearance >= 0.5)`

17. **time-to-collision with the nearest pedestrian must stay at or above 0.422 s.**
   `G(ped_ttc >= 0.422)`

