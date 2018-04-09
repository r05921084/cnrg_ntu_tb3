with open('coffee.dat', 'r') as f1:
    with open('e8n00bin', 'r') as f2:
        lines1 = f1.readlines()
        lines2 = f2.readlines()
        print len(lines1), len(lines2)

        if len(lines1) == len(lines2):
            err_cnt = 0
            for i in range(len(lines1)):
                if lines1[i] == lines2[i]:
                    pass
                else:
                    print 'disagree at line %d' % i
                    print lines1[i]
                    print '.' * 40
                    print lines2[i]
                    print '=' * 40
                    err_cnt += 1

            print 'stat: %d in %d lines, %f %%.' % (err_cnt, len(lines1), 100 * float(err_cnt) / len(lines1))
            if float(err_cnt) / len(lines1) < 0.5:
                print 'CHECK PASS'
            else:
                print 'DISAGREE'


        else:
            print 'length of two file disagree!'