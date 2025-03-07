/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 * 
 *  ==================================================================================================
 */

#include <iostream>

#include "MailSender/CurlMailSender.hpp"
#include "MailSender/Mail.hpp"

using crf::communication::mailsender::MailAddress;
using crf::communication::mailsender::Mail;
using crf::communication::mailsender::CurlMailSender;

int main() {
    MailAddress sender("test@cern.ch");
    MailAddress receiver1("pptaszni@cern.ch");
    MailAddress receiver2("pptaszni@gmail.com");

    CurlMailSender mailsender;

    mailsender.initialize();

    Mail mail(sender);
    mail.addReceiver(receiver1);
    mail.addReceiver(receiver2);
    mail.setSubject("Prova");
    mail.setContent("Dupa dupa dupa");

    mailsender.send(mail);

    return 0;
}
