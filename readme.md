# PID tuner for reaction wheel controller

## Main requirements
- Be able to change & store PID controller values from the web interface.
- Visualize the data from the RWC.

## Usage
1. Clone the repo and flash the board.
2. Turn RWC on and wait for it to initialize.
3. Connect to `platform` wifi network and go to [`http://10.0.0.1`](http://10.0.0.1).
4. Proceed with tesing & [tuning](#pid-tuning).


## PID tuning
Orientation control is divided into 2 PID controllers: vehicle angular speed and vehicle
position. It is crucial to tune the speed controller first to get
meaningfull and consistent values.

There are many ways PID controller can be tuned. The method which has proved to be working
well is from [this](https://pidexplained.com/how-to-tune-a-pid-controller/) website.
Actual vehicle configuration needs to be considered when selecting the gains:
- Configuration with relatively small D term has proven to be adequately fast in response
and stable. __Large D term can lead to instability!__
- Speed controller benefits from higher I term. P term can be decreased by a small amount,
raising the I term to compensate for it.  __Too large I term will lead to instability & oscillations!__ 

## Notes & issues
- Web interface and graphs in particular may become unstable or completely stop working.
RWC restart may help. Other browsers (tested mostly in Firefox) may work better.