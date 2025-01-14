/*
 * Copyright (c) 2014 Natale Patriciello <natale.patriciello@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#define NS_LOG_APPEND_CONTEXT                                                                      \
    {                                                                                              \
        std::clog << Simulator::Now().GetSeconds() << " ";                                         \
    }

#include "tcp-cubichyplus.h"

#include "ns3/log.h"

NS_LOG_COMPONENT_DEFINE("TcpCubichyplusplus");

namespace ns3
{

NS_OBJECT_ENSURE_REGISTERED(TcpCubichyplusplus);

TypeId
TcpCubichyplusplus::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::TcpCubichyplusplus")
            .SetParent<TcpSocketBase>()
            .AddConstructor<TcpCubichyplusplus>()
            .SetGroupName("Internet")
            .AddAttribute("FastConvergence",
                          "Enable (true) or disable (false) fast convergence",
                          BooleanValue(true),
                          MakeBooleanAccessor(&TcpCubichyplusplus::m_fastConvergence),
                          MakeBooleanChecker())
            .AddAttribute("TcpFriendliness",
                          "Enable (true) or disable (false) TCP friendliness",
                          BooleanValue(true),
                          MakeBooleanAccessor(&TcpCubichyplusplus::m_tcpFriendliness),
                          MakeBooleanChecker())
            .AddAttribute("Beta",
                          "Beta for multiplicative decrease",
                          DoubleValue(0.7),
                          MakeDoubleAccessor(&TcpCubichyplusplus::m_beta),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("HyStart",
                          "Enable (true) or disable (false) hybrid slow start algorithm",
                          BooleanValue(true),
                          MakeBooleanAccessor(&TcpCubichyplusplus::m_hystart),
                          MakeBooleanChecker())
            .AddAttribute("HyStartLowWindow",
                          "Lower bound cWnd for hybrid slow start (segments)",
                          UintegerValue(16),
                          MakeUintegerAccessor(&TcpCubichyplusplus::m_hystartLowWindow),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("HyStartDetect",
                          "Hybrid Slow Start detection mechanisms:"
                          "packet train, delay, both",
                          EnumValue(HybridSSDetectionMode::DELAY),
                          MakeEnumAccessor(&TcpCubichyplusplus::m_hystartDetect),
                          MakeEnumChecker(HybridSSDetectionMode::PACKET_TRAIN,
                                          "PACKET_TRAIN",
                                          HybridSSDetectionMode::DELAY,
                                          "DELAY",
                                          HybridSSDetectionMode::BOTH,
                                          "BOTH"))
            .AddAttribute("HyStartMinSamples",
                          "Number of delay samples for detecting the increase of delay",
                          UintegerValue(8),
                          MakeUintegerAccessor(&TcpCubichyplusplus::m_hystartMinSamples),
                          MakeUintegerChecker<uint8_t>())
            .AddAttribute("HyStartAckDelta",
                          "Spacing between ack's indicating train",
                          TimeValue(MilliSeconds(2)),
                          MakeTimeAccessor(&TcpCubichyplusplus::m_hystartAckDelta),
                          MakeTimeChecker())
            .AddAttribute("HyStartDelayMin",
                          "Minimum time for hystart algorithm",
                          TimeValue(MilliSeconds(4)),
                          MakeTimeAccessor(&TcpCubichyplusplus::m_hystartDelayMin),
                          MakeTimeChecker())
                           .AddAttribute("HyStaradtDelayMin",
                          "Minimum time for hystart algorithm",
                          TimeValue(MilliSeconds(4)),
                          MakeTimeAccessor(&TcpCubichyplusplus::MIN_RTT_THRESH),
                          MakeTimeChecker())
                           .AddAttribute("HyStartasdDelayMin",
                          "Minimum time for hystart algorithm",
                          TimeValue(MilliSeconds(16)),
                          MakeTimeAccessor(&TcpCubichyplusplus::MAX_RTT_THRESH),
                          MakeTimeChecker())
            .AddAttribute("HyStartDelayMax",
                          "Maximum time for hystart algorithm",
                          TimeValue(MilliSeconds(1000)),
                          MakeTimeAccessor(&TcpCubichyplusplus::m_hystartDelayMax),
                          MakeTimeChecker())
            .AddAttribute("CubichyplusplusDelta",
                          "Delta Time to wait after fast recovery before adjusting param",
                          TimeValue(MilliSeconds(10)),
                          MakeTimeAccessor(&TcpCubichyplusplus::m_CubichyplusplusDelta),
                          MakeTimeChecker())
            .AddAttribute("CntClamp",
                          "Counter value when no losses are detected (counter is used"
                          " when incrementing cWnd in congestion avoidance, to avoid"
                          " floating point arithmetic). It is the modulo of the (avoided)"
                          " division",
                          UintegerValue(20),
                          MakeUintegerAccessor(&TcpCubichyplusplus::m_cntClamp),
                          MakeUintegerChecker<uint8_t>())
            .AddAttribute("C",
                          "Cubichyplusplus Scaling factor",
                          DoubleValue(0.4),
                          MakeDoubleAccessor(&TcpCubichyplusplus::m_c),
                          MakeDoubleChecker<double>(0.0));
    return tid;
}

TcpCubichyplusplus::TcpCubichyplusplus()
    : TcpCongestionOps(),
      m_cWndCnt(0),
      m_lastMaxCwnd(0),
      m_bicOriginPoint(0),
      m_bicK(0.0),
      m_delayMin(Time::Min()),
      m_epochStart(Time::Min()),
      m_found(false),
      m_roundStart(Time::Min()),
      m_endSeq(0),
      m_lastAck(Time::Min()),
      m_CubichyplusplusDelta(Time::Min()),
      m_currRtt(Time::Max()),
      m_sampleCnt(0),
      m_inCss(false),
      m_css_growth_divisor(4),
      m_currMinRtt(Time::Max()),
      m_lastRoundMinRtt(Time::Max()),
      cssBaselineMinRtt(Time::Min()),
      RttThresh(Time::Min()),
      m_sampleCssCnt(0),
      CSS_ROUNDS(5),
      MIN_RTT_DIVISOR(8),
      //L(8)
      L(2147483647)
      
      
{
    NS_LOG_FUNCTION(this);
}

TcpCubichyplusplus::TcpCubichyplusplus(const TcpCubichyplusplus& sock)
    : TcpCongestionOps(sock),
      m_fastConvergence(sock.m_fastConvergence),
      m_beta(sock.m_beta),
      m_hystart(sock.m_hystart),
      m_hystartDetect(sock.m_hystartDetect),
      m_hystartLowWindow(sock.m_hystartLowWindow),
      m_hystartAckDelta(sock.m_hystartAckDelta),
      m_hystartDelayMin(sock.m_hystartDelayMin),
      m_hystartDelayMax(sock.m_hystartDelayMax),
      m_hystartMinSamples(sock.m_hystartMinSamples),
      m_initialCwnd(sock.m_initialCwnd),
      m_cntClamp(sock.m_cntClamp),
      m_c(sock.m_c),
      m_cWndCnt(sock.m_cWndCnt),
      m_lastMaxCwnd(sock.m_lastMaxCwnd),
      m_bicOriginPoint(sock.m_bicOriginPoint),
      m_bicK(sock.m_bicK),
      m_delayMin(sock.m_delayMin),
      m_epochStart(sock.m_epochStart),
      m_found(sock.m_found),
      m_roundStart(sock.m_roundStart),
      m_endSeq(sock.m_endSeq),
      m_lastAck(sock.m_lastAck),
      m_CubichyplusplusDelta(sock.m_CubichyplusplusDelta),
      m_currRtt(sock.m_currRtt),
      m_sampleCnt(sock.m_sampleCnt),
      MIN_RTT_THRESH(sock.MIN_RTT_THRESH),
      MAX_RTT_THRESH(sock.MAX_RTT_THRESH),
      m_inCss(sock.m_inCss),
      m_css_growth_divisor(sock.m_css_growth_divisor),
      m_currMinRtt(sock.m_currMinRtt),
      m_lastRoundMinRtt(sock.m_lastRoundMinRtt),
      cssBaselineMinRtt(sock.cssBaselineMinRtt),
      RttThresh(sock.RttThresh),
      m_sampleCssCnt(sock.m_sampleCssCnt),
      CSS_ROUNDS(sock.CSS_ROUNDS),
      MIN_RTT_DIVISOR(sock.MIN_RTT_DIVISOR),
      L(sock.L)
      
{
    NS_LOG_FUNCTION(this);
}

std::string
TcpCubichyplusplus::GetName() const
{
    return "TcpCubichyplusplus";
}

void
TcpCubichyplusplus::HystartReset(Ptr<const TcpSocketState> tcb)
{
    NS_LOG_FUNCTION(this);

    m_roundStart = m_lastAck = Simulator::Now();
    m_endSeq = tcb->m_highTxMark;
    m_currRtt = Time::Min();
    m_sampleCnt = 0;
}

void
TcpCubichyplusplus::cssReset(Ptr<const TcpSocketState> tcb)
{
    NS_LOG_FUNCTION(this);

    m_roundStart = m_lastAck = Simulator::Now();
    m_endSeq = tcb->m_highTxMark;
    m_currRtt = Time::Min();
    m_sampleCssCnt = 0;
}

void
TcpCubichyplusplus::IncreaseWindow(Ptr<TcpSocketState> tcb, uint32_t segmentsAcked)
{
    NS_LOG_FUNCTION(this << tcb << segmentsAcked);

    /*if (!tcb->m_isCwndLimited)
    {
        NS_LOG_DEBUG("No increase because current cwnd " << tcb->m_cWnd
                                                         << " is not limiting the flow");
        return;
    }*/

    if (tcb->m_cWnd < tcb->m_ssThresh)
    {
        if (m_hystart && tcb->m_lastAckedSeq > m_endSeq)
        {
            std::cout<<"reseting hystart as round ends"<<std::endl;
            HystartReset(tcb); //end of hystart round
            cssReset(tcb);//fix
        }

        // In Linux, the QUICKACK socket option enables the receiver to send
        // immediate acks initially (during slow start) and then transition
        // to delayed acks.  ns-3 does not implement QUICKACK, and if ack
        // counting instead of byte counting is used during slow start window
        // growth, when TcpSocket::DelAckCount==2, then the slow start will
        // not reach as large of an initial window as in Linux.  Therefore,
        // we can approximate the effect of QUICKACK by making this slow
        // start phase perform Appropriate Byte Counting (RFC 3465)
        //tcb->m_cWnd += segmentsAcked * tcb->m_segmentSize;
        if(!m_inCss)
        {
         tcb->m_cWnd += std::min(segmentsAcked, L*536)*tcb->m_segmentSize;
         segmentsAcked = 0;
        }
        
        
        if(m_inCss)
        {
           tcb->m_cWnd += (std::min(segmentsAcked, L*536))/m_css_growth_divisor;
           segmentsAcked = 0;
        }

        NS_LOG_INFO("In SlowStart, updated to cwnd " << tcb->m_cWnd << " ssthresh "
                                                     << tcb->m_ssThresh);
        std::cout<<"inside ss"<<std::endl;
    }

    if (tcb->m_cWnd >= tcb->m_ssThresh && segmentsAcked > 0)
    {
        m_cWndCnt += segmentsAcked;
        uint32_t cnt = Update(tcb, segmentsAcked);

        /* According to RFC 6356 even once the new cwnd is
         * calculated you must compare this to the number of ACKs received since
         * the last cwnd update. If not enough ACKs have been received then cwnd
         * cannot be updated.
         */
        if (m_cWndCnt >= cnt)
        {
            tcb->m_cWnd += tcb->m_segmentSize;
            m_cWndCnt -= cnt;
            std::cout<<"inside cong avoid"<<std::endl;
            NS_LOG_INFO("In CongAvoid, updated to cwnd " << tcb->m_cWnd);
        }
        else
        {
            NS_LOG_INFO("Not enough segments have been ACKed to increment cwnd."
                        "Until now "
                        << m_cWndCnt << " cnd " << cnt);
        }
    }
}

uint32_t
TcpCubichyplusplus::Update(Ptr<TcpSocketState> tcb, uint32_t segmentsAcked)
{
    NS_LOG_FUNCTION(this);
    Time t;
    uint32_t delta;
    uint32_t bicTarget;
    uint32_t cnt = 0;
    uint32_t maxCnt;
    double offs;
    uint32_t segCwnd = tcb->GetCwndInSegments();

    m_ackCnt += segmentsAcked;

    if (m_epochStart == Time::Min())
    {
        m_epochStart = Simulator::Now(); // record the beginning of an epoch
        m_ackCnt = segmentsAcked;
        m_tcpCwnd = segCwnd;

        if (m_lastMaxCwnd <= segCwnd)
        {
            NS_LOG_DEBUG("lastMaxCwnd <= m_cWnd. K=0 and origin=" << segCwnd);
            m_bicK = 0.0;
            m_bicOriginPoint = segCwnd;
        }
        else
        {
            m_bicK = std::pow((m_lastMaxCwnd - segCwnd) / m_c, 1 / 3.);
            m_bicOriginPoint = m_lastMaxCwnd;
            NS_LOG_DEBUG("lastMaxCwnd > m_cWnd. K=" << m_bicK << " and origin=" << m_lastMaxCwnd);
        }
    }

    t = Simulator::Now() + m_delayMin - m_epochStart;

    if (t.GetSeconds() < m_bicK) /* t - K */
    {
        offs = m_bicK - t.GetSeconds();
        NS_LOG_DEBUG("t=" << t.GetSeconds() << " <k: offs=" << offs);
    }
    else
    {
        offs = t.GetSeconds() - m_bicK;
        NS_LOG_DEBUG("t=" << t.GetSeconds() << " >= k: offs=" << offs);
    }

    /* Constant value taken from Experimental Evaluation of Cubichyplusplus Tcp, available at
     * eprints.nuim.ie/1716/1/Hamiltonpfldnet2007_Cubichyplusplus_final.pdf */
    delta = m_c * std::pow(offs, 3);

    NS_LOG_DEBUG("delta: " << delta);

    if (t.GetSeconds() < m_bicK)
    {
        // below origin
        bicTarget = m_bicOriginPoint - delta;
        NS_LOG_DEBUG("t < k: Bic Target: " << bicTarget);
    }
    else
    {
        // above origin
        bicTarget = m_bicOriginPoint + delta;
        NS_LOG_DEBUG("t >= k: Bic Target: " << bicTarget);
    }

    // Next the window target is converted into a cnt or count value. Cubichyplusplus will
    // wait until enough new ACKs have arrived that a counter meets or exceeds
    // this cnt value. This is how the Cubichyplusplus implementation simulates growing
    // cwnd by values other than 1 segment size.
    if (bicTarget > segCwnd)
    {
        cnt = segCwnd / (bicTarget - segCwnd);
        NS_LOG_DEBUG("target>cwnd. cnt=" << cnt);
    }
    else
    {
        cnt = 100 * segCwnd;
    }

    if (m_lastMaxCwnd == 0 && cnt > m_cntClamp)
    {
        cnt = m_cntClamp;
    }

    if (m_tcpFriendliness)
    {
        auto scale = static_cast<uint32_t>(8 * (1024 + m_beta * 1024) / 3 / (1024 - m_beta * 1024));
        delta = (segCwnd * scale) >> 3;
        while (m_ackCnt > delta)
        {
            m_ackCnt -= delta;
            m_tcpCwnd++;
        }
        if (m_tcpCwnd > segCwnd)
        {
            delta = m_tcpCwnd - segCwnd;
            maxCnt = segCwnd / delta;
            if (cnt > maxCnt)
            {
                cnt = maxCnt;
            }
        }
    }

    // The maximum rate of cwnd increase Cubichyplusplus allows is 1 packet per
    // 2 packets ACKed, meaning cwnd grows at 1.5x per RTT.
    return std::max(cnt, 2U);
}

void
TcpCubichyplusplus::PktsAcked(Ptr<TcpSocketState> tcb, uint32_t segmentsAcked, const Time& rtt)
{
    NS_LOG_FUNCTION(this << tcb << segmentsAcked << rtt);
    
     
    /* Discard delay samples right after fast recovery */
    if (m_epochStart != Time::Min() && (Simulator::Now() - m_epochStart) < m_CubichyplusplusDelta)
    {
        return;
    }

    /* first time call or link delay decreases */
    if (m_delayMin == Time::Min() || m_delayMin > rtt)
    {
        m_delayMin = rtt;
    }

    /* hystart triggers when cwnd is larger than some threshold */
    if (!m_inCss && m_hystart && tcb->m_cWnd <= tcb->m_ssThresh &&
        tcb->m_cWnd >= m_hystartLowWindow * tcb->m_segmentSize) //added a fix for triggering of hystart only
    {
        std::cout<<"trigger hystart"<<std::endl;
        m_lastRoundMinRtt=m_currMinRtt;
        m_currMinRtt=Time::Max();
        //m_sampleCnt=0; fix
        HystartUpdate(tcb, rtt);
    }
    
    if(m_inCss)
    {
       HystartCssUpdate(tcb,rtt);
    }
    
}

void
TcpCubichyplusplus::ConservativeSlowStart(Ptr<TcpSocketState> tcb, const Time& delay)
{
     m_roundStart = m_lastAck = Simulator::Now();
     //m_endSeq = tcb->m_highTxMark; recent fix
     m_currRtt = Time::Min();
     m_inCss=true;
     m_sampleCssCnt=0;
     m_lastRoundMinRtt=m_currMinRtt;
     m_currMinRtt=Time::Max();
}


void
TcpCubichyplusplus::HystartUpdate(Ptr<TcpSocketState> tcb, const Time& delay)
{
    NS_LOG_FUNCTION(this << delay);

    //std::cout<<"inside hystart"<<std::endl;

    if (!m_found)
    {
        Time now = Simulator::Now();

        /* first detection parameter - ack-train detection */
        if ((now - m_lastAck) <= m_hystartAckDelta)
        {
            m_lastAck = now;

            if ((now - m_roundStart) > m_delayMin)
            {
                if (m_hystartDetect == HybridSSDetectionMode::PACKET_TRAIN ||
                    m_hystartDetect == HybridSSDetectionMode::BOTH)
                {
                    m_found = true;
                }
            }
        }

        /* obtain the minimum delay of more than sampling packets */
        if (m_sampleCnt < m_hystartMinSamples)
        {
            if (m_currRtt == Time::Min() || m_currRtt > delay)
            {
                m_currRtt = delay;
            }
            
            m_currMinRtt=std::min(m_currMinRtt, m_currRtt);   
            ++m_sampleCnt;
        }
        //else if ((m_currMinRtt!=Time::Max())&&(m_lastRoundMinRtt!=Time::Max()))
        else if (m_currRtt > m_delayMin + HystartDelayThresh(m_delayMin))  //testing
        {
            RttThresh = std::max(MIN_RTT_THRESH, std::min(m_lastRoundMinRtt / MIN_RTT_DIVISOR, MAX_RTT_THRESH));
            
            if(m_currMinRtt >= (m_lastRoundMinRtt + RttThresh))
            {
               cssBaselineMinRtt = m_currMinRtt;
               m_found=true;
               m_inCss=true;
               //m_sampleCnt=0; fix
               ConservativeSlowStart(tcb,delay);
            }
        }

        /*
         * Either one of two conditions are met,
         * we exit from slow start immediately.
         */
        if (m_found)
        {
            std::cout<<"exit ss"<<std::endl;
            m_found=false;//recent fix
            NS_LOG_DEBUG("Exit from SS, immediately :-)");
            //tcb->m_ssThresh = tcb->m_cWnd;
        }
    }
}

void
TcpCubichyplusplus::HystartCssUpdate(Ptr<TcpSocketState> tcb, const Time& delay)
{
    NS_LOG_FUNCTION(this << delay);
    std::cout<<"inside css"<<std::endl;
    NS_LOG_WARN("css update");
    /*if (m_sampleCnt < m_hystartMinSamples)
        {
            if (m_currRtt == Time::Min() || m_currRtt > delay)
            {
                m_currRtt = delay;
            }
            
            m_currMinRtt=std::min(m_currMinRtt, m_currRtt);   
            ++m_sampleCnt;
        }*/
    if (m_sampleCssCnt < m_hystartMinSamples)
        {
            if (m_currRtt == Time::Min() || m_currRtt > delay)
            {
                m_currRtt = delay;
            }
            
            m_currMinRtt=std::min(m_currMinRtt, m_currRtt);   
            ++m_sampleCssCnt;
            std::cout<<"increasing css sample count"<<std::endl;
        }
    else if(m_currMinRtt < cssBaselineMinRtt)
    {
           cssBaselineMinRtt = Time::Max();
           m_found=false;
           m_inCss=false;//fix
    }
    
    if(m_sampleCssCnt > CSS_ROUNDS)
    {
        m_found=true;
        m_inCss=false;//fix
    }
    
    if (m_found)
        {
            std::cout<<"exit from css"<<std::endl;
            NS_LOG_DEBUG("Exit from CSS, immediately :-)");
            m_inCss=false;
            tcb->m_ssThresh = tcb->m_cWnd;
        }

}


Time
TcpCubichyplusplus::HystartDelayThresh(const Time& t) const
{
    NS_LOG_FUNCTION(this << t);

    Time ret = t;
    if (t > m_hystartDelayMax)
    {
        ret = m_hystartDelayMax;
    }
    else if (t < m_hystartDelayMin)
    {
        ret = m_hystartDelayMin;
    }

    return ret;
}

uint32_t
TcpCubichyplusplus::GetSsThresh(Ptr<const TcpSocketState> tcb, uint32_t bytesInFlight)
{
    NS_LOG_FUNCTION(this << tcb << bytesInFlight);

    uint32_t segCwnd = tcb->GetCwndInSegments();
    NS_LOG_DEBUG("Loss at cWnd=" << segCwnd
                                 << " segments in flight=" << bytesInFlight / tcb->m_segmentSize);

    /* Wmax and fast convergence */
    if (segCwnd < m_lastMaxCwnd && m_fastConvergence)
    {
        m_lastMaxCwnd = (segCwnd * (1 + m_beta)) / 2; // Section 4.6 in RFC 8312
    }
    else
    {
        m_lastMaxCwnd = segCwnd;
    }

    m_epochStart = Time::Min(); // end of epoch

    /* Formula taken from the Linux kernel */
    uint32_t ssThresh = std::max(static_cast<uint32_t>(segCwnd * m_beta), 2U) * tcb->m_segmentSize;

    NS_LOG_DEBUG("SsThresh = " << ssThresh);

    return ssThresh;
}

void
TcpCubichyplusplus::CongestionStateSet(Ptr<TcpSocketState> tcb, const TcpSocketState::TcpCongState_t newState)
{
    NS_LOG_FUNCTION(this << tcb << newState);

    if (newState == TcpSocketState::CA_LOSS)
    {
        std::cout<<"reseting due to loss"<<std::endl;
        CubichyplusplusReset(tcb);
        HystartReset(tcb);
    }
}

void
TcpCubichyplusplus::CubichyplusplusReset(Ptr<const TcpSocketState> tcb)
{
    NS_LOG_FUNCTION(this << tcb);

    m_bicOriginPoint = 0;
    m_bicK = 0;
    m_ackCnt = 0;
    m_tcpCwnd = 0;
    m_delayMin = Time::Min();
    m_found = false;

}

Ptr<TcpCongestionOps>
TcpCubichyplusplus::Fork()
{
    NS_LOG_FUNCTION(this);
    return CopyObject<TcpCubichyplusplus>(this);
}

} // namespace ns3
