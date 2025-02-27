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

#ifndef TCPCUBICHYPLUSPLUS_H
#define TCPCUBICHYPLUSPLUS_H

#include "tcp-congestion-ops.h"
#include "tcp-socket-base.h"

namespace ns3
{

/**
 * \brief The Cubichyplusplus Congestion Control Algorithm
 *
 * TCP Cubichyplusplus is a protocol that enhances the fairness property
 * of Bic while retaining its scalability and stability. The main feature is
 * that the window growth function is defined in real time in order to be independent
 * from the RTT. More specifically, the congestion window of Cubichyplusplus is determined
 * by a function of the elapsed time from the last window reduction.
 *
 * The Cubichyplusplus code is quite similar to that of Bic.
 * The main difference is located in the method Update, an edit
 * necessary for satisfying the Cubichyplusplus window growth, that can be tuned with
 * the attribute C (the Cubichyplusplus scaling factor).
 *
 * Following the Linux implementation, we included the Hybrid Slow Start,
 * that effectively prevents the overshooting of slow start
 * while maintaining a full utilization of the network. This new type of slow
 * start can be disabled through the HyStart attribute.
 *
 * Cubichyplusplus TCP is implemented and used by default in Linux kernels 2.6.19
 * and above; this version follows the implementation in Linux 3.14, which
 * slightly differ from the Cubichyplusplus paper. It also features HyStart.
 *
 * Home page:
 *      https://web.archive.org/web/20080607093013/http://netsrv.csc.ncsu.edu/twiki/bin/view/Main/BIC
 * The work starts from the implementation of Cubichyplusplus TCP in
 * Sangtae Ha, Injong Rhee and Lisong Xu,
 * "Cubichyplusplus: A New TCP-Friendly High-Speed TCP Variant"
 * in ACM SIGOPS Operating System Review, July 2008.
 * Available from:
 *  https://web.archive.org/web/20160505194415/http://netsrv.csc.ncsu.edu/export/Cubichyplusplus_a_new_tcp_2008.pdf
 *
 * Cubichyplusplus integrates a new slow start algorithm, called HyStart.
 * The details of HyStart are presented in
 *  Sangtae Ha and Injong Rhee,
 *  "Taming the Elephants: New TCP Slow Start", NCSU TechReport 2008.
 * Available from:
 *  https://web.archive.org/web/20160528233754/http://netsrv.csc.ncsu.edu/export/hystart_techreport_2008.pdf
 *
 * More information on this implementation: http://dl.acm.org/citation.cfm?id=2756518
 */
class TcpCubichyplusplus : public TcpCongestionOps
{
  public:
    /**
     * \brief Values to detect the Slow Start mode of HyStart
     */
    enum HybridSSDetectionMode
    {
        PACKET_TRAIN = 1, //!< Detection by trains of packet
        DELAY = 2,        //!< Detection by delay value
        BOTH = 3,         //!< Detection by both
    };

    /**
     * \brief Get the type ID.
     * \return the object TypeId
     */
    static TypeId GetTypeId();

    TcpCubichyplusplus();

    /**
     * Copy constructor
     * \param sock Socket to copy
     */
    TcpCubichyplusplus(const TcpCubichyplusplus& sock);

    std::string GetName() const override;
    void PktsAcked(Ptr<TcpSocketState> tcb, uint32_t segmentsAcked, const Time& rtt) override;
    void IncreaseWindow(Ptr<TcpSocketState> tcb, uint32_t segmentsAcked) override;
    uint32_t GetSsThresh(Ptr<const TcpSocketState> tcb, uint32_t bytesInFlight) override;
    void CongestionStateSet(Ptr<TcpSocketState> tcb,
                            const TcpSocketState::TcpCongState_t newState) override;

    Ptr<TcpCongestionOps> Fork() override;

  private:
    bool m_fastConvergence; //!< Enable or disable fast convergence algorithm
    bool m_tcpFriendliness; //!< Enable or disable TCP-friendliness heuristic
    double m_beta;          //!< Beta for Cubichyplusplus multiplicative increase

    bool m_hystart;                        //!< Enable or disable HyStart algorithm
    HybridSSDetectionMode m_hystartDetect; //!< Detect way for HyStart algorithm
    uint32_t m_hystartLowWindow;           //!< Lower bound cWnd for hybrid slow start (segments)
    Time m_hystartAckDelta;                //!< Spacing between ack's indicating train
    Time m_hystartDelayMin;                //!< Minimum time for hystart algorithm
    Time m_hystartDelayMax;                //!< Maximum time for hystart algorithm
    uint8_t m_hystartMinSamples; //!< Number of delay samples for detecting the increase of delay

    uint32_t m_initialCwnd; //!< Initial cWnd
    uint8_t m_cntClamp;     //!< Modulo of the (avoided) float division for cWnd

    double m_c; //!< Cubichyplusplus Scaling factor

    // Cubichyplusplus parameters
    uint32_t m_cWndCnt;        //!<  cWnd integer-to-float counter
    uint32_t m_lastMaxCwnd;    //!<  Last maximum cWnd
    uint32_t m_bicOriginPoint; //!<  Origin point of bic function
    double m_bicK;             //!<  Time to origin point from the beginning
                               //    of the current epoch (in s)
    Time m_delayMin;           //!<  Min delay
    Time m_epochStart;         //!<  Beginning of an epoch
    bool m_found;              //!<  The exit point is found?
    Time m_roundStart;         //!<  Beginning of each round
    SequenceNumber32 m_endSeq; //!<  End sequence of the round
    Time m_lastAck;            //!<  Last time when the ACK spacing is close
    Time m_CubichyplusplusDelta;         //!<  Time to wait after recovery before update
    Time m_currRtt;            //!<  Current Rtt
    uint32_t m_sampleCnt;      //!<  Count of samples for HyStart
    uint32_t m_ackCnt;         //!<  Count the number of ACKed packets
    uint32_t m_tcpCwnd;        //!<  Estimated tcp cwnd (for Reno-friendliness)
    bool m_inCss;
    uint32_t m_css_growth_divisor;
    Time m_currMinRtt;
    Time m_lastRoundMinRtt;
    Time cssBaselineMinRtt;
    Time RttThresh;
    uint32_t m_sampleCssCnt;
    uint32_t CSS_ROUNDS;
    Time MIN_RTT_THRESH;
    Time MAX_RTT_THRESH;
    uint32_t MIN_RTT_DIVISOR;
    uint32_t L; 
    

  private:
    /**
     * \brief Reset HyStart parameters
     * \param tcb Transmission Control Block of the connection
     */
    void HystartReset(Ptr<const TcpSocketState> tcb);
    void cssReset(Ptr<const TcpSocketState> tcb);

    /**
     * \brief Reset Cubichyplusplus parameters
     * \param tcb Transmission Control Block of the connection
     */
    void CubichyplusplusReset(Ptr<const TcpSocketState> tcb);

    /**
     * \brief Cubichyplusplus window update after a new ack received
     * \param tcb Transmission Control Block of the connection
     * \param segmentsAcked Segments acked
     * \returns the congestion window update counter
     */
    uint32_t Update(Ptr<TcpSocketState> tcb, uint32_t segmentsAcked);

    /**
     * \brief Update HyStart parameters
     *
     * \param tcb Transmission Control Block of the connection
     * \param delay Delay for HyStart algorithm
     */
    void HystartUpdate(Ptr<TcpSocketState> tcb, const Time& delay);
    
    void HystartCssUpdate(Ptr<TcpSocketState> tcb, const Time& delay);

    /**
     * \brief Clamp time value in a range
     *
     * The returned value is t, clamped in a range specified
     * by attributes (HystartDelayMin < t < HystartDelayMax)
     *
     * \param t Time value to clamp
     * \return t itself if it is in range, otherwise the min or max
     * value
     */
    Time HystartDelayThresh(const Time& t) const;
    
    void ConservativeSlowStart(Ptr<TcpSocketState> tcb, const Time& delay);
};

} // namespace ns3

#endif // TCPCubichyplusplus_H
