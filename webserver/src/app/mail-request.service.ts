import { Injectable } from '@angular/core';
import { Observable, of } from 'rxjs';
import { MessageService } from './message.service';
import { MailRequest } from './mail-request';
import { REQUESTS } from './mock-requests';

@Injectable({
  providedIn: 'root'
})
export class MailRequestService {

  constructor(private messageService: MessageService) { }

  getRequests(): Observable<MailRequest[]> {
    const requests = of(REQUESTS);
    this.messageService.add('MailRequestService: fetched requests');
    return requests;
  }

  getNextID(): Observable<number> {
    var maxID = -1;
    REQUESTS.forEach(function (request) {
      maxID = maxID > request.id ? maxID : request.id;
    });

    const nextID = of(++maxID);

    return nextID;
  }
}
