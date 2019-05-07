import gps
 
# Listen on port 2947 (gpsd) of localhost
session = gps.gps("localhost", "2947")
session.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)
 
while True:
    try:
        report = session.next()
        # Wait for a 'TPV' report and capture/print desired values
        # Here we do so with the longitude and lattitude
        # Printing the full report will show 
        if report['class'] == 'TPV':
            if (hasattr(report, 'lon') & hasattr(report,'lat')):
                lattitude = report.lat
                longitude = report.lon
                print(lattitude, longitude)
    except KeyError:
        pass
    except KeyboardInterrupt:
        quit()
    except StopIteration:
        session = None
        print("GPSD has terminated")